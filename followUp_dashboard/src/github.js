// Small GitHub REST client that collects everything the dashboard shows.
// Uses Node's built-in fetch (Node 18+), so no dependencies are required.

const DAY = 864e5;
const HEATMAP_DAYS = 26 * 7;
const ACTIVE_DAYS = 14; // a branch with a commit in the last 14 days counts as "active"

export class GitHubError extends Error {
  constructor(message, status) {
    super(message);
    this.status = status;
  }
}

export class GitHubClient {
  constructor({ token, apiUrl }) {
    this.token = token;
    this.apiUrl = apiUrl;
    // Conditional-request cache: with a token, GitHub does not count
    // "304 Not Modified" answers against the rate limit, so refreshes are cheap.
    this.etags = new Map();
    this.rateLimit = null;
  }

  headers() {
    const h = {
      Accept: 'application/vnd.github+json',
      'X-GitHub-Api-Version': '2022-11-28',
      'User-Agent': 'github-progress-dashboard',
    };
    if (this.token) h.Authorization = `Bearer ${this.token}`;
    return h;
  }

  // Returns { status, link, data }.
  async request(pathOrUrl) {
    const url = pathOrUrl.startsWith('http') ? pathOrUrl : this.apiUrl + pathOrUrl;
    const headers = this.headers();
    const hit = this.etags.get(url);
    if (hit) headers['If-None-Match'] = hit.etag;

    const res = await fetch(url, { headers });
    this.trackRateLimit(res);
    if (res.status === 304 && hit) return hit;

    if (!res.ok) {
      let message = res.statusText;
      try { message = (await res.json()).message || message; } catch { /* ignore */ }
      if ((res.status === 403 || res.status === 429) && res.headers.get('x-ratelimit-remaining') === '0') {
        const reset = Number(res.headers.get('x-ratelimit-reset')) * 1000;
        message = `GitHub rate limit reached. It resets at ${new Date(reset).toLocaleTimeString()}.` +
          (this.token ? '' : ' Add a GITHUB_TOKEN to get a much higher limit.');
      } else if (res.status === 404) {
        message = 'Not found. Check the repository name, or add a GITHUB_TOKEN with access if it is private.';
      } else if (res.status === 401) {
        message = 'GitHub rejected the token (401). Check GITHUB_TOKEN.';
      }
      throw new GitHubError(message, res.status);
    }

    const out = {
      status: res.status,
      link: res.headers.get('link') || '',
      data: res.status === 204 ? null : await res.json(),
    };
    const etag = res.headers.get('etag');
    if (etag) {
      if (this.etags.size > 1000) this.etags.delete(this.etags.keys().next().value);
      this.etags.set(url, { ...out, etag });
    }
    return out;
  }

  trackRateLimit(res) {
    const remaining = res.headers.get('x-ratelimit-remaining');
    const resource = res.headers.get('x-ratelimit-resource');
    if (remaining === null || resource !== 'core') return;
    this.rateLimit = {
      remaining: Number(remaining),
      limit: Number(res.headers.get('x-ratelimit-limit')),
      reset: new Date(Number(res.headers.get('x-ratelimit-reset')) * 1000).toISOString(),
    };
  }

  async json(path) {
    const { data } = await this.request(path);
    return data ?? [];
  }

  // Counts items in a list endpoint without downloading them all:
  // ask for 1 item per page and read the "last" page number from the Link header.
  async count(path) {
    const sep = path.includes('?') ? '&' : '?';
    let res;
    try {
      res = await this.request(`${path}${sep}per_page=1`);
    } catch (err) {
      if (err.status === 409) return 0; // empty repository
      throw err;
    }
    const last = res.link.split(',').find((part) => /rel="last"/.test(part));
    if (last) {
      const url = last.match(/<([^>]+)>/)?.[1];
      if (url) return Number(new URL(url).searchParams.get('page')) || 0;
    }
    return Array.isArray(res.data) ? res.data.length : 0;
  }

  async search(q, perPage = 1) {
    const data = await this.json(
      `/search/issues?q=${encodeURIComponent(q)}&sort=updated&order=desc&per_page=${perPage}`
    );
    return { total: data.total_count ?? 0, items: data.items ?? [] };
  }

  // Commit history of one branch since a date. Returns { commits, truncated }.
  async commitsSince(full, branch, sinceIso, maxPages = 5) {
    const commits = [];
    for (let page = 1; page <= maxPages; page++) {
      let batch;
      try {
        batch = await this.json(
          `/repos/${full}/commits?sha=${encodeURIComponent(branch)}&since=${sinceIso}&per_page=100&page=${page}`
        );
      } catch (err) {
        if (err.status === 409) return { commits, truncated: false };
        throw err;
      }
      commits.push(...batch);
      if (batch.length < 100) return { commits, truncated: false };
    }
    return { commits, truncated: true };
  }

  async compare(full, base, head) {
    const ref = (b) => encodeURIComponent(b).replace(/%2F/gi, '/');
    const data = await this.json(`/repos/${full}/compare/${ref(base)}...${ref(head)}?per_page=1`);
    return { ahead: data.ahead_by ?? 0, behind: data.behind_by ?? 0 };
  }

  async ciSummary(full) {
    const [workflows, runs] = await Promise.all([
      this.json(`/repos/${full}/actions/workflows?per_page=50`),
      this.json(`/repos/${full}/actions/runs?per_page=50`),
    ]);
    const list = runs.workflow_runs || [];
    const finished = list.filter((r) => r.status === 'completed' && r.conclusion !== 'skipped' && r.conclusion !== 'cancelled');
    const passed = finished.filter((r) => r.conclusion === 'success').length;
    const mapRun = (r) => ({
      id: r.id, name: r.name, title: r.display_title, branch: r.head_branch, event: r.event,
      status: r.status, conclusion: r.conclusion, url: r.html_url,
      date: r.run_started_at || r.created_at, actor: r.actor?.login, avatar: r.actor?.avatar_url,
      durationSec: r.status === 'completed' && r.run_started_at
        ? Math.max(0, Math.round((new Date(r.updated_at) - new Date(r.run_started_at)) / 1000)) : null,
    });
    return {
      totalRuns: runs.total_count ?? list.length,
      sampleSize: finished.length,
      successRate: finished.length ? Math.round((passed / finished.length) * 100) : null,
      workflows: (workflows.workflows || []).map((w) => {
        const last = list.find((r) => r.workflow_id === w.id);
        return {
          name: w.name, path: w.path, enabled: w.state === 'active', url: w.html_url,
          lastRun: last ? mapRun(last) : null,
        };
      }),
      recentRuns: list.slice(0, 8).map(mapRun),
    };
  }

  async repoSummary(full, { activityWeeks = 12 } = {}) {
    const warnings = [];
    const safe = async (label, promise, fallback) => {
      try { return await promise; } catch (err) {
        warnings.push(`${label}: ${err.message}`);
        return fallback;
      }
    };

    const info = await this.json(`/repos/${full}`);
    const defaultBranch = info.default_branch;
    const createdAt = info.created_at;

    const [
      commitCount, branchCount, branches, contributors,
      merged, closedPrs, closedIssues, openPrs, openIssues, milestones, languages, ci,
    ] = await Promise.all([
      safe('Commit count', this.count(`/repos/${full}/commits?sha=${encodeURIComponent(defaultBranch)}`), null),
      safe('Branch count', this.count(`/repos/${full}/branches`), null),
      safe('Branches', this.json(`/repos/${full}/branches?per_page=100`), []),
      safe('Contributors', this.json(`/repos/${full}/contributors?per_page=30`), []),
      safe('Merged PRs', this.search(`repo:${full} is:pr is:merged`, 10), { total: null, items: [] }),
      safe('Closed PRs', this.search(`repo:${full} is:pr is:closed is:unmerged`, 5), { total: null, items: [] }),
      safe('Closed issues', this.search(`repo:${full} is:issue is:closed`, 10), { total: null, items: [] }),
      safe('Open PRs', this.search(`repo:${full} is:pr is:open`, 10), { total: null, items: [] }),
      safe('Open issues', this.search(`repo:${full} is:issue is:open`, 10), { total: null, items: [] }),
      safe('Milestones', this.json(`/repos/${full}/milestones?state=all&sort=due_on&per_page=20`), []),
      safe('Languages', this.json(`/repos/${full}/languages`), {}),
      safe('GitHub Actions', this.ciSummary(full), null),
    ]);

    // ── History of every branch (deduplicated), so feature-branch work counts too ──
    const trackedBranches = [...branches]
      .sort((a, b) => (b.name === defaultBranch) - (a.name === defaultBranch))
      .slice(0, 30);
    const branchData = await Promise.all(trackedBranches.map(async (b) => {
      const isDefault = b.name === defaultBranch;
      const [history, cmp] = await Promise.all([
        safe(`History of ${b.name}`, this.commitsSince(full, b.name, createdAt, isDefault ? 10 : 3), { commits: [], truncated: false }),
        isDefault ? null : safe(`Compare ${b.name}`, this.compare(full, defaultBranch, b.name), null),
      ]);
      return { branch: b, isDefault, history, cmp };
    }));

    const unique = new Map();
    let truncated = false;
    for (const { history, branch } of branchData) {
      truncated ||= history.truncated;
      for (const c of history.commits) {
        if (!unique.has(c.sha)) unique.set(c.sha, { ...c, branches: [branch.name] });
        else unique.get(c.sha).branches.push(branch.name);
      }
    }
    const allCommits = [...unique.values()].sort((a, b) => new Date(commitDate(b)) - new Date(commitDate(a)));
    const defaultCommits = branchData.find((d) => d.isDefault)?.history.commits || [];
    const weekStart = startOfWeek(new Date());

    const branchDetails = branchData.map(({ branch, isDefault, history, cmp }) => {
      const last = history.commits[0];
      const lastDate = last ? commitDate(last) : null;
      let status = 'idle';
      if (isDefault) status = 'default';
      else if (cmp && cmp.ahead === 0) status = 'merged';
      else if (lastDate && Date.now() - new Date(lastDate) < ACTIVE_DAYS * DAY) status = 'active';
      return {
        name: branch.name, isDefault, protected: branch.protected, status,
        url: `${info.html_url}/tree/${encodeURIComponent(branch.name).replace(/%2F/gi, '/')}`,
        ahead: cmp?.ahead ?? null, behind: cmp?.behind ?? null,
        commits: history.commits.length,
        lastCommit: last ? {
          sha: last.sha.slice(0, 7), message: firstLine(last), url: last.html_url,
          author: authorName(last), avatar: last.author?.avatar_url || null, date: lastDate,
        } : null,
      };
    }).sort((a, b) => (b.isDefault - a.isDefault) || (new Date(b.lastCommit?.date || 0) - new Date(a.lastCommit?.date || 0)));

    // Number of distinct people who committed on any branch
    const team = new Set(allCommits.map((c) => c.author?.login || authorName(c)));

    const languageTotal = Object.values(languages).reduce((a, b) => a + b, 0);

    return {
      repo: {
        fullName: info.full_name,
        description: info.description,
        url: info.html_url,
        defaultBranch,
        private: info.private,
        stars: info.stargazers_count,
        forks: info.forks_count,
        language: info.language,
        topics: info.topics || [],
        createdAt,
        pushedAt: info.pushed_at,
        owner: { login: info.owner?.login, avatar: info.owner?.avatar_url },
      },
      counts: {
        commits: commitCount,
        commitsAllBranches: truncated ? `${allCommits.length}+` : allCommits.length,
        branches: branchCount,
        activeBranches: branchDetails.filter((b) => b.status === 'active').length,
        contributors: team.size || contributors.length,
        mergedPrs: merged.total,
        closedPrs: closedPrs.total,
        closedIssues: closedIssues.total,
        openPrs: openPrs.total,
        openIssues: openIssues.total,
        commitsThisWeek: allCommits.filter((c) => new Date(commitDate(c)) >= weekStart).length,
        ciRuns: ci?.totalRuns ?? null,
        ciSuccessRate: ci?.successRate ?? null,
      },
      activity: weeklyBuckets(defaultCommits, activityWeeks),
      activityAll: weeklyBuckets(allCommits, activityWeeks),
      daily: dailyBuckets(allCommits, HEATMAP_DAYS),
      done: {
        mergedPrs: merged.items.map(mapIssue),
        closedIssues: closedIssues.items.map(mapIssue),
      },
      inProgress: {
        openPrs: openPrs.items.map(mapIssue),
        openIssues: openIssues.items.map(mapIssue),
      },
      closedPrs: closedPrs.items.map(mapIssue),
      milestones: milestones.map((m) => {
        const total = m.open_issues + m.closed_issues;
        return {
          title: m.title, url: m.html_url, state: m.state, dueOn: m.due_on, description: m.description,
          open: m.open_issues, closed: m.closed_issues,
          percent: total ? Math.round((m.closed_issues / total) * 100) : 0,
        };
      }),
      languages: Object.entries(languages).map(([name, bytes]) => ({
        name, bytes, percent: languageTotal ? (bytes / languageTotal) * 100 : 0,
      })),
      ci,
      branches: branchDetails,
      recentCommits: allCommits.slice(0, 12).map((c) => ({
        sha: c.sha.slice(0, 7),
        message: firstLine(c),
        author: authorName(c),
        avatar: c.author?.avatar_url || null,
        date: commitDate(c),
        url: c.html_url,
        branches: c.branches,
      })),
      warnings,
      fetchedAt: new Date().toISOString(),
    };
  }
}

function mapIssue(i) {
  return {
    number: i.number, title: i.title, url: i.html_url, user: i.user?.login, avatar: i.user?.avatar_url,
    date: i.pull_request?.merged_at || i.closed_at || i.created_at, createdAt: i.created_at,
    draft: Boolean(i.draft), labels: (i.labels || []).map((l) => ({ name: l.name, color: l.color })),
    comments: i.comments,
  };
}

function commitDate(c) {
  return c.commit?.author?.date || c.commit?.committer?.date;
}

function authorName(c) {
  return c.author?.login || c.commit?.author?.name || 'unknown';
}

function firstLine(c) {
  return (c.commit?.message || '').split('\n')[0];
}

function startOfWeek(d) {
  const x = new Date(Date.UTC(d.getUTCFullYear(), d.getUTCMonth(), d.getUTCDate()));
  const day = (x.getUTCDay() + 6) % 7; // Monday = 0
  x.setUTCDate(x.getUTCDate() - day);
  return x;
}

function weeklyBuckets(commits, weeks) {
  const first = startOfWeek(new Date(Date.now() - (weeks - 1) * 7 * DAY));
  const buckets = Array.from({ length: weeks }, (_, i) => ({
    weekStart: new Date(first.getTime() + i * 7 * DAY).toISOString().slice(0, 10),
    commits: 0,
  }));
  for (const c of commits) {
    const idx = Math.floor((startOfWeek(new Date(commitDate(c))) - first) / (7 * DAY));
    if (idx >= 0 && idx < weeks) buckets[idx].commits++;
  }
  return buckets;
}

// One bucket per day, ending today; the first bucket is a Monday so the
// front-end can draw a calendar heatmap with full week columns.
function dailyBuckets(commits, days) {
  const today = new Date();
  const end = Date.UTC(today.getUTCFullYear(), today.getUTCMonth(), today.getUTCDate());
  const first = startOfWeek(new Date(end - (days - 1) * DAY)).getTime();
  const n = Math.round((end - first) / DAY) + 1;
  const buckets = Array.from({ length: n }, (_, i) => ({ date: new Date(first + i * DAY).toISOString().slice(0, 10), commits: 0 }));
  for (const c of commits) {
    const d = new Date(commitDate(c));
    const idx = Math.round((Date.UTC(d.getUTCFullYear(), d.getUTCMonth(), d.getUTCDate()) - first) / DAY);
    if (idx >= 0 && idx < n) buckets[idx].commits++;
  }
  return buckets;
}
