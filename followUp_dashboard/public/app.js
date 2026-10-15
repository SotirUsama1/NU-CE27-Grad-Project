// Dashboard front-end. Talks only to our own server (/api/...), never to
// GitHub directly, so the GitHub token never reaches the browser.
const ALL = '__all__';
const DAY = 864e5;
const $ = (id) => document.getElementById(id);
const state = {
  config: null, current: null, summary: null, overview: null,
  activityMode: 'all', workTab: null, lastFetch: null, failed: false,
};
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmt = (n) => (n === null || n === undefined ? '—' : typeof n === 'number' ? n.toLocaleString() : n);
const plural = (n, word) => `${fmt(n)} ${word}${n === 1 ? '' : 's'}`;
const shortDate = (iso, opts = {}) => new Date(iso).toLocaleDateString(undefined, { month: 'short', day: 'numeric', ...opts });
const longDate = (iso) => shortDate(iso, { year: 'numeric' });
const daysBetween = (a, b) => Math.floor((new Date(b) - new Date(a)) / DAY);

function ago(iso) {
  if (!iso) return '';
  const s = Math.round((Date.now() - new Date(iso)) / 1000);
  if (s < 60) return 'just now';
  const m = s / 60, h = m / 60, d = h / 24;
  if (m < 60) return `${Math.floor(m)}m ago`;
  if (h < 24) return `${Math.floor(h)}h ago`;
  if (d < 30) return `${Math.floor(d)}d ago`;
  if (d < 365) return `${Math.floor(d / 30)}mo ago`;
  return `${Math.floor(d / 365)}y ago`;
}

async function api(path) {
  const res = await fetch(path);
  const body = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(body.error || `Request failed (${res.status})`);
  return body;
}

// ── Icons (inline SVG, stroke-based) ───────────────────────────────────────
const ICONS = {
  commit: '<circle cx="12" cy="12" r="3.5"/><path d="M3 12h5.5M15.5 12H21"/>',
  week: '<rect x="3" y="5" width="18" height="16" rx="2"/><path d="M3 10h18M8 3v4M16 3v4"/>',
  branch: '<circle cx="6" cy="5" r="2.2"/><circle cx="6" cy="19" r="2.2"/><circle cx="18" cy="7" r="2.2"/><path d="M6 7.2v9.6M18 9.2c0 5-6 4-11 7.6"/>',
  merge: '<circle cx="6" cy="5" r="2.2"/><circle cx="6" cy="19" r="2.2"/><circle cx="18" cy="15" r="2.2"/><path d="M6 7.2v9.6M6 7.2c0 5 4 7.8 9.8 7.8"/>',
  pr: '<circle cx="6" cy="5" r="2.2"/><circle cx="6" cy="19" r="2.2"/><circle cx="18" cy="19" r="2.2"/><path d="M6 7.2v9.6M18 16.8V9a3 3 0 0 0-3-3h-4"/><path d="m13 3.5-2.5 2.5 2.5 2.5"/>',
  issue: '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="1.5"/>',
  shield: '<path d="M12 2 4 5v6c0 5 3.4 9.4 8 11 4.6-1.6 8-6 8-11V5z"/><path d="m8.5 12 2.5 2.5 4.5-5"/>',
  play: '<circle cx="12" cy="12" r="9"/><path d="m10 8.5 5 3.5-5 3.5z"/>',
  users: '<circle cx="9" cy="8" r="3.5"/><path d="M2.5 20c.8-3.6 3.4-5.5 6.5-5.5s5.7 1.9 6.5 5.5"/><path d="M16 4.6a3.5 3.5 0 0 1 0 6.8M18.5 14.8c1.6.8 2.6 2.6 3 5.2"/>',
  repo: '<path d="M5 4.5A2.5 2.5 0 0 1 7.5 2H20v16H7.5A2.5 2.5 0 0 0 5 20.5z"/><path d="M5 20.5A2.5 2.5 0 0 0 7.5 23H20v-5"/>',
  clock: '<circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/>',
  lock: '<rect x="5" y="11" width="14" height="10" rx="2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/>',
  flag: '<path d="M5 21V4M5 4h11l-2 4 2 4H5"/>',
  ok: '<path d="m5 12.5 4.5 4.5L19 7.5"/>',
  fail: '<path d="M6 6l12 12M18 6 6 18"/>',
  running: '<path d="M12 3a9 9 0 1 0 9 9"/>',
  dash: '<path d="M6 12h12"/>',
  alert: '<path d="M12 7v6M12 17h.01"/>',
  star: '<path d="m12 3 2.7 5.6 6.1.8-4.5 4.2 1.1 6-5.4-2.9-5.4 2.9 1.1-6L3.2 9.4l6.1-.8z"/>',
};
const icon = (name) => `<svg viewBox="0 0 24 24" aria-hidden="true">${ICONS[name] || ''}</svg>`;
const statusChip = (kind, label, iconName) =>
  `<span class="status ${kind}${iconName === 'running' ? ' running' : ''}">${icon(iconName)}${esc(label)}</span>`;

function emptyState(iconName, title, text = '') {
  return `<div class="empty">${icon(iconName)}<strong>${title}</strong>${text ? `<span>${text}</span>` : ''}</div>`;
}

// ── Alerts ─────────────────────────────────────────────────────────────────
let baseWarnings = [];
function alerts(list) {
  $('alerts').innerHTML = list.map(([kind, msg]) => `<div class="alert ${kind}"><div>${msg}</div></div>`).join('');
}
const baseAlerts = (extra = []) => alerts([...baseWarnings, ...extra]);

// ── Tooltip (one shared element, driven by data-tip attributes) ───────────
const tip = $('tooltip');
function showTip(html, x, y) {
  tip.innerHTML = html;
  tip.hidden = false;
  const r = tip.getBoundingClientRect();
  let left = x + 14, top = y + 14;
  if (left + r.width > innerWidth - 8) left = x - r.width - 14;
  if (top + r.height > innerHeight - 8) top = y - r.height - 14;
  tip.style.left = `${Math.max(8, left)}px`;
  tip.style.top = `${Math.max(8, top)}px`;
}
const hideTip = () => { tip.hidden = true; };
function onPointer(e) {
  const t = e.target.closest?.('[data-tip]');
  if (t) showTip(t.dataset.tip, e.clientX, e.clientY); else hideTip();
}
document.addEventListener('pointermove', onPointer);
document.addEventListener('pointerdown', onPointer);
document.addEventListener('focusin', (e) => {
  const t = e.target.closest?.('[data-tip]');
  if (!t) return hideTip();
  const r = t.getBoundingClientRect();
  showTip(t.dataset.tip, r.left + r.width / 2, r.top);
});
document.addEventListener('scroll', hideTip, { passive: true });
// Returns attribute-safe HTML; the values themselves are escaped first.
const tipHtml = (title, value, extra = '') =>
  esc(`<div class="t-title">${esc(title)}</div><div class="t-value">${esc(value)}</div>${extra ? `<div class="t-title">${esc(extra)}</div>` : ''}`);

// ── Count-up numbers ───────────────────────────────────────────────────────
function countUp(el) {
  const to = Number(el.dataset.to);
  const from = Number(el.dataset.from || 0);
  if (reducedMotion || !Number.isFinite(to) || to === from) { el.textContent = fmt(to); return; }
  const start = performance.now(), dur = 900;
  const step = (now) => {
    const p = Math.min(1, (now - start) / dur);
    const eased = 1 - (1 - p) ** 3;
    el.textContent = fmt(Math.round(from + (to - from) * eased));
    if (p < 1) requestAnimationFrame(step);
  };
  requestAnimationFrame(step);
}
function animateNumbers(root) {
  root.querySelectorAll('[data-to]').forEach(countUp);
}
// Keeps the previously shown value so refreshes animate from old → new
const prevValues = new Map();
function numberSpan(key, value, suffix = '') {
  if (typeof value !== 'number') return `${fmt(value)}${suffix ? `<small>${suffix}</small>` : ''}`;
  const from = prevValues.get(key) ?? 0;
  prevValues.set(key, value);
  return `<span data-to="${value}" data-from="${from}">${fmt(from)}</span>${suffix ? `<small>${suffix}</small>` : ''}`;
}

// ── Hero ───────────────────────────────────────────────────────────────────
function setTitle(full) {
  const idx = full.indexOf(':');
  const eyebrow = idx > 0 ? full.slice(0, idx).trim() : 'Project progress';
  const title = idx > 0 ? full.slice(idx + 1).trim() : full;
  $('eyebrow').textContent = eyebrow;
  $('title').textContent = title;
  document.title = full;
}

function projectStart(s) {
  return state.config.project?.start || s?.repo?.createdAt || null;
}

function heroProgress(s) {
  const total = s.milestones.reduce((a, m) => a + m.open + m.closed, 0);
  if (total) {
    const closed = s.milestones.reduce((a, m) => a + m.closed, 0);
    return { pct: Math.round((closed / total) * 100), cap: 'Milestones complete', note: `${closed} of ${total} milestone tasks closed` };
  }
  const { end } = state.config.project || {};
  const start = projectStart(s);
  if (end && start) {
    const pct = Math.round(Math.min(1, Math.max(0, (Date.now() - new Date(start)) / (new Date(end) - new Date(start)))) * 100);
    return { pct, cap: 'Timeline elapsed', note: `${plural(Math.max(0, daysBetween(new Date(), end)), 'day')} to the deadline` };
  }
  const features = s.branches.filter((b) => !b.isDefault && b.ahead !== null);
  if (features.length) {
    const merged = features.filter((b) => b.status === 'merged').length;
    return {
      pct: Math.round((merged / features.length) * 100), cap: 'Branches integrated',
      note: `${merged} of ${plural(features.length, 'feature branch').replace(/branchs$/, 'branches')} fully merged into ${s.repo.defaultBranch}`,
    };
  }
  return null;
}

function renderRing(progress) {
  if (!progress) { $('heroRing').innerHTML = ''; return; }
  const r = 84, c = 2 * Math.PI * r;
  const prevPct = prevValues.get('ring') ?? 0;
  $('heroRing').innerHTML = `
    <div class="ring" role="img" aria-label="${esc(progress.cap)}: ${progress.pct}%">
      <svg viewBox="0 0 196 196">
        <defs><linearGradient id="ringGrad" x1="0" y1="0" x2="1" y2="1"><stop offset="0" class="g0"/><stop offset="1" class="g1"/></linearGradient></defs>
        <circle class="track" cx="98" cy="98" r="${r}" fill="none" stroke-width="12"/>
        <circle class="value" cx="98" cy="98" r="${r}" fill="none" stroke="url(#ringGrad)" stroke-width="12" stroke-linecap="round"
          stroke-dasharray="${c}" stroke-dashoffset="${c * (1 - prevPct / 100)}"/>
      </svg>
      <div class="ring-center">
        <div class="pct">${numberSpan('ring', progress.pct, '%')}</div>
        <div class="cap">${esc(progress.cap)}</div>
      </div>
    </div>
    <div class="ring-note">${esc(progress.note)}</div>`;
  const arc = $('heroRing').querySelector('.value');
  requestAnimationFrame(() => requestAnimationFrame(() => { arc.style.strokeDashoffset = c * (1 - progress.pct / 100); }));
  animateNumbers($('heroRing'));
}

function renderTimeline(start) {
  const { end } = state.config.project || {};
  if (!start) { $('timeline').innerHTML = ''; return; }
  const day = Math.max(1, daysBetween(start, new Date()) + 1);
  if (end) {
    const pct = Math.min(100, Math.max(0, ((Date.now() - new Date(start)) / (new Date(end) - new Date(start))) * 100));
    const left = daysBetween(new Date(), end);
    $('timeline').innerHTML = `
      <div class="tl-row">
        <span>Kick-off <strong>${longDate(start)}</strong></span>
        <span>Day <strong>${fmt(day)}</strong> · ${Math.round(pct)}% of the schedule used</span>
        <span>Deadline <strong>${longDate(end)}</strong> · ${left >= 0 ? `${plural(left, 'day')} left` : `${plural(-left, 'day')} overdue`}</span>
      </div>
      <div class="tl-bar"><div class="tl-fill" style="width:0"></div><div class="tl-now" style="left:0"></div></div>`;
    requestAnimationFrame(() => requestAnimationFrame(() => {
      $('timeline').querySelector('.tl-fill').style.width = `${pct}%`;
      $('timeline').querySelector('.tl-now').style.left = `${pct}%`;
    }));
  } else {
    $('timeline').innerHTML = `
      <div class="tl-row">
        <span>Kick-off <strong>${longDate(start)}</strong></span>
        <span>Day <strong>${fmt(day)}</strong> of development</span>
        <span>Set <code>PROJECT_END_DATE</code> in <code>.env</code> for a deadline countdown</span>
      </div>
      <div class="tl-bar tl-open"></div>`;
  }
}

function renderHero(s) {
  const r = s.repo;
  if (r.description) $('heroDesc').textContent = r.description;
  const langs = s.languages.slice().sort((a, b) => b.bytes - a.bytes).slice(0, 3).map((l) => l.name);
  $('heroMeta').innerHTML = `
    <a class="meta-chip" href="${esc(r.url)}" target="_blank" rel="noopener">
      ${icon('repo')}${esc(r.fullName)}
    </a>
    ${r.private ? `<span class="meta-chip">${icon('lock')}Private</span>` : ''}
    <span class="meta-chip">${icon('branch')}${esc(r.defaultBranch)}</span>
    ${langs.length ? `<span class="meta-chip">${esc(langs.join(' · '))}</span>` : ''}
    <span class="meta-chip">${icon('clock')}Last push ${ago(r.pushedAt)}</span>`;

  const start = projectStart(s);
  const features = s.branches.filter((b) => !b.isDefault);
  const stats = [
    ['days', start ? Math.max(1, daysBetween(start, new Date()) + 1) : null, 'Days in development'],
    ['team', s.counts.contributors, 'Team members'],
    ['tracks', features.length, 'Feature tracks'],
    ['active', s.counts.activeBranches, 'Active this fortnight'],
  ];
  $('heroStats').innerHTML = stats.map(([k, v, l]) => `
    <div class="hero-stat"><div class="v">${numberSpan(`hero-${k}`, v)}</div><div class="l">${l}</div></div>`).join('');
  animateNumbers($('heroStats'));
  renderRing(heroProgress(s));
  renderTimeline(start);
}

// ── KPI tiles ──────────────────────────────────────────────────────────────
function sparkline(values) {
  if (!values || values.length < 2) return '';
  const W = 84, H = 30, max = Math.max(1, ...values);
  const pts = values.map((v, i) => [(i / (values.length - 1)) * (W - 4) + 2, H - 3 - (v / max) * (H - 8)]);
  const line = pts.map((p, i) => `${i ? 'L' : 'M'}${p[0].toFixed(1)} ${p[1].toFixed(1)}`).join(' ');
  const last = pts[pts.length - 1];
  return `<svg class="spark" viewBox="0 0 ${W} ${H}" aria-hidden="true">
    <path class="area" d="${line} L${last[0].toFixed(1)} ${H} L2 ${H} Z"/>
    <path class="line" d="${line}"/><circle cx="${last[0]}" cy="${last[1]}" r="3.5"/></svg>`;
}

function renderKpis(items, loading = false) {
  $('kpis').innerHTML = items.map((k, i) => `
    <div class="kpi tone-${k.tone}" style="animation-delay:${i * 40}ms">
      <div class="kpi-top"><span class="kpi-icon">${icon(k.icon)}</span><span class="label">${k.label}</span></div>
      <div class="value">${loading ? '<span class="skeleton">000</span>' : numberSpan(`kpi-${k.label}`, k.value, k.suffix)}</div>
      <div class="sub">${loading ? '&nbsp;' : k.sub || ''}</div>
      ${loading ? '' : sparkline(k.spark)}
    </div>`).join('');
  if (!loading) animateNumbers($('kpis'));
}

function kpiItems(c, s) {
  const weekly = s?.activityAll?.map((w) => w.commits);
  const ci = c.ciSuccessRate;
  return [
    { label: 'Commits', icon: 'commit', tone: 'blue', value: c.commitsAllBranches ?? c.commits, sub: `all branches · ${fmt(c.commits)} on main line`, spark: weekly },
    { label: 'This week', icon: 'week', tone: 'cyan', value: c.commitsThisWeek, sub: 'commits since Monday' },
    { label: 'Branches', icon: 'branch', tone: 'violet', value: c.branches, sub: `${fmt(c.activeBranches ?? 0)} active in last 14 days` },
    { label: 'Merged PRs', icon: 'merge', tone: 'green', value: c.mergedPrs, sub: 'work integrated' },
    { label: 'Open PRs', icon: 'pr', tone: 'amber', value: c.openPrs, sub: 'in review' },
    { label: 'Issues', icon: 'issue', tone: 'pink', value: c.openIssues, sub: `open · ${fmt(c.closedIssues)} closed` },
    { label: 'CI pass rate', icon: 'shield', tone: 'teal', value: ci, suffix: ci === null || ci === undefined ? '' : '%', sub: ci === null || ci === undefined ? 'no finished runs yet' : 'recent workflow runs' },
    { label: 'Pipeline runs', icon: 'play', tone: 'indigo', value: c.ciRuns, sub: 'GitHub Actions, all time' },
  ];
}

// ── Weekly commit column chart (plain SVG) ─────────────────────────────────
function niceStep(max) {
  const raw = max / 4;
  const p = 10 ** Math.floor(Math.log10(raw || 1));
  return [1, 2, 5, 10].map((m) => m * p).find((s) => s >= raw) || 1;
}

function columnPath(x, y, w, h, r) {
  r = Math.min(r, h, w / 2);
  if (h <= 0) return '';
  return `M${x} ${y + h}V${y + r}Q${x} ${y} ${x + r} ${y}H${x + w - r}Q${x + w} ${y} ${x + w} ${y + r}V${y + h}Z`;
}

function renderBarChart(el, data, animate = true) {
  el._data = data;
  if (!data || !data.length) { el.innerHTML = emptyState('commit', 'No activity data yet'); return; }
  const W = Math.max(280, el.clientWidth || 720), H = Math.max(250, Math.min(420, el.clientHeight || 250)), padL = 34, padR = 6, padT = 24, padB = 30;
  const max = Math.max(1, ...data.map((d) => d.commits));
  const step = niceStep(max);
  const top = Math.ceil(max / step) * step;
  const cw = (W - padL - padR) / data.length;
  const bw = Math.min(24, cw * 0.6);
  const y = (v) => padT + (H - padT - padB) * (1 - v / top);
  const base = y(0);
  const maxIdx = data.reduce((best, d, i) => (d.commits >= data[best].commits ? i : best), 0);
  const labelEvery = Math.ceil(data.length / Math.max(2, Math.floor((W - padL) / 54)));

  let svg = `<svg width="${W}" height="${H}" viewBox="0 0 ${W} ${H}" role="img" aria-label="Commits per week">`;
  for (let v = step; v <= top; v += step) {
    svg += `<line class="grid-line" x1="${padL}" x2="${W - padR}" y1="${y(v)}" y2="${y(v)}"/>`;
    svg += `<text class="axis" x="${padL - 8}" y="${y(v) + 4}" text-anchor="end">${v}</text>`;
  }
  svg += `<text class="axis" x="${padL - 8}" y="${base + 4}" text-anchor="end">0</text>`;
  data.forEach((d, i) => {
    const x = padL + i * cw + (cw - bw) / 2;
    const label = shortDate(d.weekStart + 'T00:00:00Z', { timeZone: 'UTC' });
    const tipText = tipHtml(`Week of ${label}`, plural(d.commits, 'commit'));
    svg += `<g class="col" data-tip="${tipText}" tabindex="0" aria-label="Week of ${label}: ${plural(d.commits, 'commit')}">
      <rect class="hover-bg" x="${padL + i * cw}" y="${padT - 8}" width="${cw}" height="${base - padT + 8}" rx="6" fill="transparent"/>
      ${d.commits ? `<path class="bar${animate ? ' bar-grow' : ''}" style="animation-delay:${i * 35}ms" d="${columnPath(x, y(d.commits), bw, base - y(d.commits), 4)}"/>` : ''}
    </g>`;
    const isLast = i === data.length - 1;
    if (d.commits && (i === maxIdx || isLast)) svg += `<text class="val" x="${x + bw / 2}" y="${y(d.commits) - 7}" text-anchor="middle">${d.commits}</text>`;
    if (i % labelEvery === 0 || isLast) svg += `<text class="axis" x="${x + bw / 2}" y="${H - 8}" text-anchor="middle">${isLast ? 'This wk' : label}</text>`;
  });
  svg += `<line class="baseline" x1="${padL}" x2="${W - padR}" y1="${base}" y2="${base}"/>`;
  el.innerHTML = svg + '</svg>';
}

// Re-draw charts when their container changes width
const resizer = new ResizeObserver((entries) => {
  for (const e of entries) {
    const el = e.target;
    const size = `${el.clientWidth}x${el.clientHeight}`;
    if (!el._data || el._size === size) continue;
    el._size = size;
    renderBarChart(el, el._data, false);
  }
});
['activityChart', 'overviewChart'].forEach((id) => resizer.observe($(id)));

function renderActivity() {
  const s = state.summary;
  if (!s) return;
  document.querySelectorAll('#activityTabs .tab').forEach((t) => t.classList.toggle('active', t.dataset.activity === state.activityMode));
  const data = state.activityMode === 'all' ? s.activityAll : s.activity;
  renderBarChart($('activityChart'), data);
  const total = data.reduce((a, d) => a + d.commits, 0);
  $('activityNote').textContent = `${plural(total, 'commit')} in the last ${data.length} weeks · ${state.activityMode === 'all' ? 'every branch' : s.repo.defaultBranch}`;

  const n = data.length;
  const best = data.reduce((a, d) => (d.commits > a.commits ? d : a), data[0] || { commits: 0 });
  const thisWeek = data[n - 1]?.commits ?? 0, lastWeek = data[n - 2]?.commits ?? 0;
  const delta = thisWeek - lastWeek;
  const trend = delta === 0 ? '' : `<span class="trend ${delta > 0 ? 'up' : 'down'}">${delta > 0 ? '▲' : '▼'} ${Math.abs(delta)} vs last week</span>`;
  $('activityFoot').innerHTML = `
    <div><div class="v">${(total / Math.max(1, n)).toFixed(1)}</div><div class="l">average commits / week</div></div>
    <div><div class="v">${fmt(best.commits)}</div><div class="l">best week${best.commits ? ` · ${shortDate(best.weekStart + 'T00:00:00Z', { timeZone: 'UTC' })}` : ''}</div></div>
    <div><div class="v">${fmt(thisWeek)}${trend}</div><div class="l">this week so far</div></div>`;
}

// ── Contribution heatmap ───────────────────────────────────────────────────
function renderHeatmap(daily) {
  if (!daily?.length) { $('heatmap').innerHTML = emptyState('week', 'No activity yet'); $('heatStats').innerHTML = ''; return; }
  const cell = 16, gap = 3, padL = 30, padT = 20;
  const weeks = Math.ceil(daily.length / 7);
  const W = padL + weeks * (cell + gap), H = padT + 7 * (cell + gap);
  const max = Math.max(...daily.map((d) => d.commits));
  const level = (v) => (!v ? 0 : max <= 1 ? 4 : Math.min(4, Math.ceil((v / max) * 4)));
  // Scales down to fit narrow screens, never up past its natural size
  let svg = `<svg viewBox="0 0 ${W} ${H}" style="max-width:${W * 1.25}px" role="img" aria-label="Daily commits heatmap">`;
  ['Mon', '', 'Wed', '', 'Fri', '', ''].forEach((d, i) => {
    if (d) svg += `<text class="lbl" x="0" y="${padT + i * (cell + gap) + cell - 3}">${d}</text>`;
  });
  let lastMonth = -1;
  daily.forEach((d, i) => {
    const w = Math.floor(i / 7), dow = i % 7;
    const date = new Date(d.date + 'T00:00:00Z');
    if (dow === 0 && date.getUTCMonth() !== lastMonth) {
      lastMonth = date.getUTCMonth();
      if (w < weeks - 1) svg += `<text class="lbl" x="${padL + w * (cell + gap)}" y="11">${date.toLocaleDateString(undefined, { month: 'short', timeZone: 'UTC' })}</text>`;
    }
    const label = date.toLocaleDateString(undefined, { weekday: 'short', month: 'short', day: 'numeric', year: 'numeric', timeZone: 'UTC' });
    svg += `<rect class="cell h${level(d.commits)}" x="${padL + w * (cell + gap)}" y="${padT + dow * (cell + gap)}" width="${cell}" height="${cell}"
      data-tip="${tipHtml(label, plural(d.commits, 'commit'))}"/>`;
  });
  $('heatmap').innerHTML = svg + '</svg>';

  const activeDays = daily.filter((d) => d.commits).length;
  let longest = 0, run = 0;
  for (const d of daily) { run = d.commits ? run + 1 : 0; longest = Math.max(longest, run); }
  const best = daily.reduce((a, d) => (d.commits > a.commits ? d : a), daily[0]);
  $('heatStats').innerHTML = `
    <div class="heat-stat"><div class="v">${numberSpan('heat-active', activeDays)}</div><div class="l">active days</div></div>
    <div class="heat-stat"><div class="v">${numberSpan('heat-streak', longest)}</div><div class="l">longest streak (days)</div></div>
    <div class="heat-stat"><div class="v">${numberSpan('heat-best', best.commits)}</div><div class="l">${best.commits ? `busiest day · ${shortDate(best.date + 'T00:00:00Z', { timeZone: 'UTC' })}` : 'busiest day'}</div></div>`;
  animateNumbers($('heatStats'));
}

// ── CI / CD ────────────────────────────────────────────────────────────────
function runStatus(run) {
  if (!run) return statusChip('neutral', 'No runs', 'dash');
  if (run.status !== 'completed') return statusChip('info', run.status === 'queued' ? 'Queued' : 'Running', 'running');
  switch (run.conclusion) {
    case 'success': return statusChip('good', 'Passed', 'ok');
    case 'failure': case 'timed_out': case 'startup_failure': return statusChip('bad', 'Failed', 'fail');
    case 'action_required': return statusChip('warn', 'Needs action', 'alert');
    case 'cancelled': return statusChip('neutral', 'Cancelled', 'dash');
    case 'skipped': return statusChip('neutral', 'Skipped', 'dash');
    default: return statusChip('neutral', run.conclusion || 'Unknown', 'dash');
  }
}
const duration = (sec) => (sec === null || sec === undefined ? '' : sec < 60 ? `${sec}s` : `${Math.floor(sec / 60)}m ${sec % 60}s`);

function renderCi(ci, repoUrl) {
  $('actionsLink').href = `${repoUrl}/actions`;
  if (!ci || (!ci.workflows.length && !ci.recentRuns.length)) {
    $('ci').innerHTML = emptyState('play', 'No GitHub Actions workflows', 'Add a workflow in <code>.github/workflows</code> to see builds and tests here.');
    return;
  }
  const r = 34, c = 2 * Math.PI * r, rate = ci.successRate ?? 0;
  const tone = ci.successRate === null ? 'var(--muted)' : rate >= 80 ? 'var(--good)' : rate >= 50 ? 'var(--warn)' : 'var(--bad)';
  const verdict = ci.successRate === null ? 'No finished runs yet' : rate >= 80 ? 'Pipelines healthy' : rate >= 50 ? 'Some pipelines failing' : 'Pipelines need attention';
  const enabled = ci.workflows.filter((w) => w.enabled).length;
  $('ci').innerHTML = `
    <div class="ci-top">
      <div class="ci-gauge" role="img" aria-label="CI pass rate ${ci.successRate ?? 'unknown'}%">
        <svg viewBox="0 0 84 84"><circle class="track" cx="42" cy="42" r="${r}" fill="none" stroke-width="8"/>
          <circle class="value" cx="42" cy="42" r="${r}" fill="none" stroke="${tone}" stroke-width="8" stroke-linecap="round" stroke-dasharray="${c}" stroke-dashoffset="${c}"/></svg>
        <div class="center">${ci.successRate === null ? '—' : `${rate}%`}</div>
      </div>
      <div class="ci-summary">
        <div class="big">${verdict}</div>
        <div class="sm">${ci.sampleSize ? `pass rate of the last ${plural(ci.sampleSize, 'finished run')}` : ''}</div>
        <div class="sm">${plural(ci.workflows.length, 'workflow')} · ${enabled} enabled · ${fmt(ci.totalRuns)} runs total</div>
      </div>
    </div>
    <div class="section-label">Workflows</div>
    <div class="wf-list">${ci.workflows.map((w) => `
      <div class="wf">
        ${w.enabled ? runStatus(w.lastRun) : statusChip('neutral', 'Disabled', 'dash')}
        <a class="name" href="${esc(w.url)}" target="_blank" rel="noopener" title="${esc(w.path)}">${esc(w.name)}</a>
        <span class="when">${w.lastRun ? ago(w.lastRun.date) : ''}</span>
      </div>`).join('')}
    </div>
    <div class="section-label">Recent runs</div>
    <ul class="runs">${ci.recentRuns.slice(0, 5).map((run) => `
      <li>${runStatus(run)}
        <div class="r-main"><a class="r-title" href="${esc(run.url)}" target="_blank" rel="noopener">${esc(run.title || run.name)}</a>
        <span class="r-meta"><span class="mono">${esc(run.branch)}</span> · ${esc(run.name)}${run.durationSec !== null ? ` · ${duration(run.durationSec)}` : ''} · ${ago(run.date)}</span></div>
      </li>`).join('')}
    </ul>`;
  const arc = $('ci').querySelector('.ci-gauge .value');
  requestAnimationFrame(() => requestAnimationFrame(() => { arc.style.strokeDashoffset = c * (1 - rate / 100); }));
}

// ── Feature tracks (branches) ──────────────────────────────────────────────
const BRANCH_STATUS = {
  default: ['neutral', 'Main line', 'star'],
  active: ['info', 'Active', 'running'],
  merged: ['good', 'Merged', 'ok'],
  idle: ['neutral', 'Idle', 'clock'],
};

function renderTracks(s) {
  const list = s.branches;
  $('branchNote').textContent = `${plural(list.length, 'branch').replace(/branchs$/, 'branches')} · compared with ${s.repo.defaultBranch}`;
  if (!list.length) { $('branches').innerHTML = emptyState('branch', 'No branches'); return; }
  const maxDiff = Math.max(1, ...list.map((b) => Math.max(b.ahead || 0, b.behind || 0)));
  $('branches').innerHTML = list.map((b) => {
    const [kind, label, ic] = BRANCH_STATUS[b.status] || BRANCH_STATUS.idle;
    const chip = `<span class="status ${kind}">${icon(ic === 'running' ? 'ok' : ic)}${label}</span>`;
    const lc = b.lastCommit;
    const diff = b.isDefault
      ? `<div class="diff-nums"><span>${plural(b.commits, 'commit')}</span><span>base</span></div>`
      : `<div class="diff-bar" data-tip="${tipHtml(b.name, `${b.ahead ?? '?'} ahead · ${b.behind ?? '?'} behind`, `compared with ${s.repo.defaultBranch}`)}">
           <div class="half left"><span class="behind" style="width:${((b.behind || 0) / maxDiff) * 100}%"></span></div>
           <div class="mid"></div>
           <div class="half"><span class="ahead" style="width:${((b.ahead || 0) / maxDiff) * 100}%"></span></div>
         </div>
         <div class="diff-nums"><span>${fmt(b.behind)} behind</span><span>${fmt(b.ahead)} ahead</span></div>`;
    return `
      <div class="track">
        <div class="track-head">${chip}<a class="track-name" href="${esc(b.url)}" target="_blank" rel="noopener" title="${esc(b.name)}">${esc(b.name)}</a>
          ${b.protected ? `<span class="muted" title="Protected branch">${icon('lock').replace('<svg', '<svg width="12" height="12" fill="none" stroke="currentColor" stroke-width="2"')}</span>` : ''}</div>
        <div class="track-commit">${lc ? `
          <a class="msg" href="${esc(lc.url)}" target="_blank" rel="noopener" title="${esc(lc.message)}">${esc(lc.message)}</a>
          <span style="white-space:nowrap">· ${ago(lc.date)}</span>` : 'No commits'}</div>
        <div class="track-diff">${diff}</div>
      </div>`;
  }).join('');
}

// ── Pull requests & issues ─────────────────────────────────────────────────
function workTabs(s) {
  return [
    { id: 'openPrs', label: 'Open PRs', items: s.inProgress.openPrs, count: s.counts.openPrs, chip: (i) => i.draft ? statusChip('neutral', 'Draft', 'dash') : statusChip('info', 'In review', 'pr'), verb: 'opened', icon: 'pr' },
    { id: 'mergedPrs', label: 'Merged', items: s.done.mergedPrs, count: s.counts.mergedPrs, chip: () => statusChip('good', 'Merged', 'ok'), verb: 'merged', icon: 'merge' },
    { id: 'closedPrs', label: 'Closed', items: s.closedPrs, count: s.counts.closedPrs, chip: () => statusChip('neutral', 'Closed', 'fail'), verb: 'closed', icon: 'pr' },
    { id: 'openIssues', label: 'Open issues', items: s.inProgress.openIssues, count: s.counts.openIssues, chip: () => statusChip('info', 'To do', 'issue'), verb: 'opened', icon: 'issue' },
    { id: 'closedIssues', label: 'Closed issues', items: s.done.closedIssues, count: s.counts.closedIssues, chip: () => statusChip('good', 'Done', 'ok'), verb: 'closed', icon: 'issue' },
  ];
}

function renderWork() {
  const s = state.summary;
  if (!s) return;
  const tabs = workTabs(s);
  if (!state.workTab) state.workTab = (tabs.find((t) => t.items.length) || tabs[0]).id;
  $('workTabs').innerHTML = tabs.map((t) =>
    `<button class="tab ${t.id === state.workTab ? 'active' : ''}" data-work="${t.id}" role="tab">${t.label}<span class="count">${fmt(t.count)}</span></button>`).join('');
  const tab = tabs.find((t) => t.id === state.workTab);
  $('workList').innerHTML = tab.items.length
    ? tab.items.map((i) => `
      <li>
        <div class="main">
          <a class="title" href="${esc(i.url)}" target="_blank" rel="noopener">${esc(i.title)}</a>
          <span class="meta"><span>#${i.number}</span><span>· ${tab.verb} ${ago(tab.verb === 'opened' ? i.createdAt : i.date)}</span>
            ${i.labels.slice(0, 3).map((l) => `<span class="label-chip"><i style="background:#${esc(l.color)}"></i>${esc(l.name)}</span>`).join('')}</span>
        </div>
        ${tab.chip(i)}
      </li>`).join('')
    : `<li style="border:0">${emptyState(tab.icon, `No ${tab.label.toLowerCase()}`, tab.id.includes('Issue') ? 'Track tasks as GitHub issues so finished work shows up here.' : '')}</li>`;
}

// ── Latest commits ─────────────────────────────────────────────────────────
function renderCommits(s) {
  const def = s.repo.defaultBranch;
  $('commitList').innerHTML = s.recentCommits.length
    ? s.recentCommits.slice(0, 7).map((c) => {
      const onMain = c.branches.includes(def);
      const shown = onMain ? [def] : c.branches.slice(0, 2);
      const more = onMain ? 0 : c.branches.length - shown.length;
      return `
        <li>
          <span class="dot"></span>
          <div style="min-width:0">
            <a class="msg" href="${esc(c.url)}" target="_blank" rel="noopener" title="${esc(c.message)}">${esc(c.message)}</a>
            <div class="meta"><span class="sha">${esc(c.sha)}</span><span>· ${ago(c.date)}</span>
              ${shown.map((b) => `<span class="branch-chip" title="${esc(b)}">${esc(b)}</span>`).join('')}${more > 0 ? `<span class="muted">+${more}</span>` : ''}</div>
          </div>
        </li>`;
    }).join('')
    : `<li style="display:block">${emptyState('commit', 'No commits yet')}</li>`;
}

// ── Milestones ─────────────────────────────────────────────────────────────
function renderMilestones(s) {
  $('milestoneLink').href = `${s.repo.url}/milestones`;
  $('milestones').innerHTML = s.milestones.length
    ? s.milestones.map((m) => `
      <div class="milestone">
        <div class="row">
          <a href="${esc(m.url)}" target="_blank" rel="noopener">${esc(m.title)}</a>
          <span class="pct">${m.percent}%</span>
        </div>
        <div class="progress"><span style="width:${m.percent}%"></span></div>
        <div class="meta"><span>${m.closed} done · ${m.open} open</span>${m.dueOn ? `<span>· due ${longDate(m.dueOn)}</span>` : ''}
          ${m.state === 'closed' ? statusChip('good', 'Completed', 'ok') : ''}</div>
      </div>`).join('')
    : emptyState('flag', 'No milestones yet',
      `Create milestones on GitHub (e.g. <em>Simulation world</em>, <em>Multi-agent planner</em>, <em>Final demo</em>) and attach issues to them. Their progress will appear here and drive the progress ring. <a href="${esc(s.repo.url)}/milestones/new" target="_blank" rel="noopener">Create one ↗</a>`);
}

// ── Languages ──────────────────────────────────────────────────────────────
function renderLanguages(s) {
  let langs = s.languages.slice().sort((a, b) => b.bytes - a.bytes);
  if (langs.length > 7) {
    const rest = langs.slice(6);
    langs = [...langs.slice(0, 6), { name: 'Other', bytes: rest.reduce((a, l) => a + l.bytes, 0), percent: rest.reduce((a, l) => a + l.percent, 0) }];
  }
  const facts = `
    <div class="repo-facts">
      <div><strong>${longDate(s.repo.createdAt)}</strong>repository created</div>
      <div><strong>${fmt(s.repo.stars)}</strong>stars</div>
      <div><strong>${fmt(s.repo.forks)}</strong>forks</div>
    </div>`;
  if (!langs.length) { $('languages').innerHTML = emptyState('repo', 'No code detected yet') + facts; return; }
  const pctText = (p) => (p < 0.1 ? '<0.1' : p.toFixed(1));
  $('languages').innerHTML = `
    <div class="lang-bar" role="img" aria-label="${esc(langs.map((l) => `${l.name} ${pctText(l.percent)}%`).join(', '))}">
      ${langs.map((l, i) => `<span style="flex:${Math.max(l.percent, 0.6)};background:var(--c${i + 1})" data-tip="${tipHtml(l.name, `${pctText(l.percent)}%`, `${fmt(l.bytes)} bytes`)}"></span>`).join('')}
    </div>
    <ul class="lang-list">${langs.map((l, i) => `<li><i class="sw" style="background:var(--c${i + 1})"></i>${esc(l.name)}<span class="pct">${pctText(l.percent)}%</span></li>`).join('')}</ul>
    ${facts}`;
}

// ── Single repository view ─────────────────────────────────────────────────
function renderRepo(s) {
  $('heroRing').classList.remove('hidden');
  $('mainTab').textContent = s.repo.defaultBranch;
  renderHero(s);
  renderKpis(kpiItems(s.counts, s));
  renderActivity();
  renderCi(s.ci, s.repo.url);
  renderHeatmap(s.daily);
  renderTracks(s);
  renderWork();
  renderCommits(s);
  renderMilestones(s);
  renderLanguages(s);
  renderRate(s.rateLimit);
  baseAlerts(s.warnings.map((w) => ['warn', esc(w)]));
}

// ── All repositories view ──────────────────────────────────────────────────
function renderOverview(data) {
  const ok = data.repos.filter((r) => !r.error);
  const sum = (k) => ok.reduce((a, r) => a + (typeof r.counts[k] === 'number' ? r.counts[k] : parseInt(r.counts[k], 10) || 0), 0);
  const counts = {
    commits: sum('commits'), commitsAllBranches: sum('commitsAllBranches'), branches: sum('branches'), activeBranches: sum('activeBranches'),
    commitsThisWeek: sum('commitsThisWeek'), mergedPrs: sum('mergedPrs'), closedIssues: sum('closedIssues'),
    openPrs: sum('openPrs'), openIssues: sum('openIssues'), ciRuns: sum('ciRuns'), ciSuccessRate: null,
  };
  const weeks = ok[0]?.activity.map((w, i) => ({
    weekStart: w.weekStart, commits: ok.reduce((a, r) => a + (r.activity[i]?.commits || 0), 0),
  })) || [];
  renderKpis(kpiItems(counts, { activityAll: weeks }));

  $('heroMeta').innerHTML = `<span class="meta-chip">${icon('repo')}${plural(data.repos.length, 'repository').replace(/repositorys$/, 'repositories')}</span>`;
  $('heroStats').innerHTML = [
    ['ov-commits', counts.commitsAllBranches, 'Commits'], ['ov-branches', counts.branches, 'Branches'],
    ['ov-active', counts.activeBranches, 'Active branches'], ['ov-merged', counts.mergedPrs, 'Merged PRs'],
  ].map(([k, v, l]) => `<div class="hero-stat"><div class="v">${numberSpan(k, v)}</div><div class="l">${l}</div></div>`).join('');
  animateNumbers($('heroStats'));
  $('heroRing').innerHTML = '';
  renderTimeline(state.config.project?.start || null);

  renderBarChart($('overviewChart'), weeks);
  const cols = [['commitsAllBranches', 'Commits'], ['branches', 'Branches'], ['commitsThisWeek', 'This week'], ['mergedPrs', 'Merged PRs'],
    ['openPrs', 'Open PRs'], ['openIssues', 'Open issues'], ['ciSuccessRate', 'CI pass %']];
  $('overviewTable').innerHTML = `
    <thead><tr><th>Repository</th>${cols.map(([, l]) => `<th class="n">${l}</th>`).join('')}<th>Last push</th></tr></thead>
    <tbody>${data.repos.map((r) => r.error
      ? `<tr><td>${esc(r.repo)}</td><td colspan="${cols.length + 1}" class="muted">${esc(r.error)}</td></tr>`
      : `<tr><td><a href="#" data-repo="${esc(r.repo)}">${esc(r.repo)}</a></td>${cols.map(([k]) => `<td class="n">${fmt(r.counts[k])}</td>`).join('')}<td class="muted">${ago(r.pushedAt)}</td></tr>`).join('')}
    </tbody>
    <tfoot><tr><td>Total</td>${cols.map(([k]) => `<td class="n">${k === 'ciSuccessRate' ? '' : fmt(sum(k))}</td>`).join('')}<td></td></tr></tfoot>`;
  renderRate(data.rateLimit);
  baseAlerts(data.repos.filter((r) => r.error).map((r) => ['error', `${esc(r.repo)}: ${esc(r.error)}`]));
}

// ── Deployment (AWS connection) panel ──────────────────────────────────────
function renderAws(aws) {
  $('awsBadge').className = `badge ${aws.connected ? 'on' : 'off'}`;
  $('awsBadge').textContent = aws.connected ? 'Running on AWS' : 'Running locally';
  const tokenSource = { env: '.env / environment variable', 'aws-secrets-manager': 'AWS Secrets Manager', none: 'Not set' }[aws.tokenSource] || aws.tokenSource;
  $('awsBody').innerHTML = `
    <dl class="aws-grid">
      <div><dt>Deploy target</dt><dd>${esc(aws.deployTarget)}</dd></div>
      <div><dt>AWS region</dt><dd>${esc(aws.region || 'Not set')}</dd></div>
      <div><dt>Token source</dt><dd>${esc(tokenSource)}</dd></div>
      <div><dt>Secrets Manager secret</dt><dd>${esc(aws.secretName || 'Not set')}</dd></div>
      <div><dt>Public URL</dt><dd>${aws.publicUrl ? `<a href="${esc(aws.publicUrl)}" target="_blank" rel="noopener">${esc(aws.publicUrl)}</a>` : 'Not set'}</dd></div>
    </dl>
    ${aws.connected ? '' : `
    <p class="muted" style="margin:0 0 6px">To host this dashboard on AWS (full guide in <code>docs/AWS.md</code>):</p>
    <ol>
      <li>Store your GitHub token in AWS Secrets Manager.</li>
      <li>Build the Docker image (<code>docker build -t github-dashboard .</code>) and push it to Amazon ECR.</li>
      <li>In the ECS console choose <strong>Express mode</strong>, pick the image, set container port <code>8080</code> and health check path <code>/health</code>.</li>
      <li>Add environment variables: <code>GITHUB_REPOS</code>, <code>DASHBOARD_PASSWORD</code>, <code>DEPLOY_TARGET=ecs</code>, and <code>GITHUB_TOKEN</code> as a <em>Secret</em>.</li>
      <li>Open the Application URL ECS gives you, and put it in <code>PUBLIC_URL</code>.</li>
    </ol>`}`;
}

function renderRate(rl) {
  $('rateInfo').textContent = rl
    ? `GitHub API: ${fmt(rl.remaining)} / ${fmt(rl.limit)} requests left this hour`
    : '';
}

// ── Loading & events ───────────────────────────────────────────────────────
function setLive(kind, text) {
  $('liveStatus').className = `live ${kind}`;
  $('updated').textContent = text;
}

async function load(force = false) {
  const repo = state.current;
  if (!repo) return;
  $('refreshBtn').disabled = true;
  if (!state.summary && !state.overview) renderKpis(kpiItems({}, null), true);
  try {
    const q = force ? '&refresh=1' : '';
    if (repo === ALL) {
      $('repoView').classList.add('hidden');
      $('overviewView').classList.remove('hidden');
      state.overview = await api(`/api/overview?x=1${q}`);
      if (state.current === ALL) renderOverview(state.overview);
    } else {
      $('overviewView').classList.add('hidden');
      $('repoView').classList.remove('hidden');
      const summary = await api(`/api/summary?repo=${encodeURIComponent(repo)}${q}`);
      if (state.current === repo) { state.summary = summary; renderRepo(summary); }
    }
    state.lastFetch = new Date();
    state.failed = false;
    updateClock();
  } catch (err) {
    state.failed = true;
    setLive('err', 'Connection problem');
    baseAlerts([['error', esc(err.message)]]);
    if (!state.summary && !state.overview) renderKpis(kpiItems({}, null));
  } finally {
    $('refreshBtn').disabled = false;
  }
}

function updateClock() {
  if (state.failed || !state.lastFetch) return;
  setLive('on', `Live · updated ${ago(state.lastFetch.toISOString())}`);
}

function setupTheme() {
  $('themeBtn').addEventListener('click', () => {
    const current = document.documentElement.dataset.theme || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    const next = current === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem('theme', next); } catch { /* storage blocked */ }
  });
}

async function init() {
  setupTheme();
  if (reducedMotion) $('roverMotion')?.remove();
  try {
    state.config = await api('/api/config');
  } catch (err) {
    setLive('err', 'Server unreachable');
    alerts([['error', `Could not reach the dashboard server: ${esc(err.message)}`]]);
    return;
  }
  const { config } = state;
  setTitle(config.title);
  renderAws(config.aws);

  baseWarnings = [];
  if (!config.tokenConfigured) baseWarnings.push(['info', 'Running without a <code>GITHUB_TOKEN</code>: public data works, but GitHub allows only 60 requests per hour. Add a read-only token to <code>.env</code> for faster, more frequent refreshes.']);
  if (!config.repos.length) {
    alerts([...baseWarnings, ['info', 'No repositories configured yet. Add them to <code>GITHUB_REPOS</code> in your <code>.env</code> file, e.g. <code>GITHUB_REPOS=SotirUsama1/NU-CE27-Grad-Project</code>, then restart.']]);
    $('kpis').innerHTML = '';
    setLive('', 'Not configured');
    return;
  }

  const sel = $('repoSelect');
  if (config.repos.length > 1) {
    $('repoSelectWrap').classList.remove('hidden');
    sel.innerHTML = `<option value="${ALL}">All repositories (${config.repos.length})</option>` +
      config.repos.map((r) => `<option value="${esc(r)}">${esc(r)}</option>`).join('');
  }
  const fromHash = decodeURIComponent(location.hash.slice(1));
  state.current = [ALL, ...config.repos].includes(fromHash) ? fromHash : config.repos[0];
  sel.value = state.current;

  sel.addEventListener('change', () => {
    state.current = sel.value; state.summary = null; state.overview = null; state.workTab = null;
    location.hash = sel.value; load();
  });
  $('refreshBtn').addEventListener('click', () => load(true));
  $('activityTabs').addEventListener('click', (e) => {
    const t = e.target.closest('[data-activity]');
    if (t) { state.activityMode = t.dataset.activity; renderActivity(); }
  });
  $('workTabs').addEventListener('click', (e) => {
    const t = e.target.closest('[data-work]');
    if (t) { state.workTab = t.dataset.work; renderWork(); }
  });
  $('overviewTable').addEventListener('click', (e) => {
    const a = e.target.closest('a[data-repo]');
    if (!a) return;
    e.preventDefault();
    sel.value = a.dataset.repo; sel.dispatchEvent(new Event('change'));
  });

  setInterval(updateClock, 30_000);
  setInterval(() => { if (!document.hidden) load(); }, Math.max(60, config.refreshSeconds) * 1000); // auto-refresh
  load();
}

init();
