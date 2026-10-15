// GitHub Progress Dashboard – local server
// Start with:  npm start   (then open http://localhost:3000)
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadEnvFile, getConfig } from './src/config.js';
import { GitHubClient } from './src/github.js';
import { loadSecretsFromAws, awsStatus, setTokenSource } from './src/aws.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = path.join(__dirname, 'public');

loadEnvFile(path.join(__dirname, '.env'));
if (process.env.GITHUB_TOKEN) setTokenSource('env');
await loadSecretsFromAws(); // no-op unless AWS_SECRET_NAME is set

const config = getConfig();
const github = new GitHubClient({ token: config.githubToken, apiUrl: config.githubApiUrl });
const cache = new Map();
const inflight = new Map();

// Without a token GitHub allows only 60 requests/hour (one full load uses ~25),
// so cache longer and throttle the Refresh button to avoid hitting the limit.
const cacheTtlSeconds = config.githubToken ? config.cacheTtlSeconds : Math.max(config.cacheTtlSeconds, 1800);
const minForceSeconds = config.githubToken ? 30 : 900;

async function cached(key, ttlSeconds, force, fn) {
  const hit = cache.get(key);
  const age = hit ? (Date.now() - hit.time) / 1000 : Infinity;
  if (hit && age < (force ? minForceSeconds : ttlSeconds)) return hit.data;
  // Share one GitHub fetch between viewers who refresh at the same moment
  if (inflight.has(key)) return inflight.get(key);
  const promise = fn()
    .then((data) => { cache.set(key, { time: Date.now(), data }); return data; })
    .finally(() => inflight.delete(key));
  inflight.set(key, promise);
  return promise;
}

function getSummary(repo, force) {
  return cached(`summary:${repo}`, cacheTtlSeconds, force, () =>
    github.repoSummary(repo, { activityWeeks: config.activityWeeks })
  );
}

const MIME = {
  '.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png',
  '.ico': 'image/x-icon', '.json': 'application/json',
};

function sendJson(res, status, body) {
  res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' });
  res.end(JSON.stringify(body));
}

function checkAuth(req, res) {
  if (!config.dashboardPassword) return true;
  const header = req.headers.authorization || '';
  const [user, pass] = Buffer.from(header.replace(/^Basic /, ''), 'base64').toString().split(':');
  if (user === config.dashboardUser && pass === config.dashboardPassword) return true;
  res.writeHead(401, { 'WWW-Authenticate': 'Basic realm="GitHub Dashboard"' });
  res.end('Authentication required');
  return false;
}

function serveStatic(req, res, pathname) {
  const rel = pathname === '/' ? 'index.html' : pathname.replace(/^\/+/, '');
  const file = path.normalize(path.join(PUBLIC_DIR, rel));
  if (!file.startsWith(PUBLIC_DIR)) return sendJson(res, 403, { error: 'Forbidden' });
  fs.readFile(file, (err, data) => {
    if (err) return sendJson(res, 404, { error: 'Not found' });
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream' });
    res.end(data);
  });
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, 'http://localhost');
  const { pathname } = url;

  // Health check for AWS load balancers / App Runner (no auth)
  if (pathname === '/health') return sendJson(res, 200, { ok: true });
  if (!checkAuth(req, res)) return;

  try {
    if (pathname === '/api/config') {
      return sendJson(res, 200, {
        title: config.dashboardTitle,
        repos: config.repos,
        tokenConfigured: Boolean(config.githubToken),
        refreshSeconds: cacheTtlSeconds,
        project: { start: config.projectStart, end: config.projectEnd },
        aws: awsStatus(),
      });
    }

    if (pathname === '/api/summary') {
      const repo = url.searchParams.get('repo');
      if (!config.repos.includes(repo)) {
        return sendJson(res, 400, { error: `Repository "${repo}" is not in GITHUB_REPOS.` });
      }
      const data = await getSummary(repo, url.searchParams.has('refresh'));
      return sendJson(res, 200, { ...data, rateLimit: github.rateLimit });
    }

    if (pathname === '/api/overview') {
      const force = url.searchParams.has('refresh');
      const results = await Promise.all(config.repos.map(async (repo) => {
        try {
          const s = await getSummary(repo, force);
          return { repo, counts: s.counts, url: s.repo.url, pushedAt: s.repo.pushedAt, activity: s.activityAll };
        } catch (err) {
          return { repo, error: err.message };
        }
      }));
      return sendJson(res, 200, { repos: results, fetchedAt: new Date().toISOString(), rateLimit: github.rateLimit });
    }

    if (pathname.startsWith('/api/')) return sendJson(res, 404, { error: 'Unknown endpoint' });
    return serveStatic(req, res, pathname);
  } catch (err) {
    console.error(err);
    return sendJson(res, err.status && err.status < 600 ? err.status : 500, { error: err.message });
  }
});

server.listen(config.port, config.host, () => {
  console.log(`\n  GitHub Progress Dashboard running at http://localhost:${config.port}\n`);
  if (!config.repos.length) console.warn('  ⚠  No repositories configured. Set GITHUB_REPOS in .env');
  if (!config.githubToken) console.warn('  ⚠  No GITHUB_TOKEN set – only public repos, 60 requests/hour.');
});
