// Loads settings from a local .env file (no external packages needed) and
// exposes a single config object used by the rest of the app.
import fs from 'node:fs';
import path from 'node:path';

export function loadEnvFile(file = path.resolve(process.cwd(), '.env')) {
  if (!fs.existsSync(file)) return false;
  const lines = fs.readFileSync(file, 'utf8').split(/\r?\n/);
  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith('#')) continue;
    const eq = trimmed.indexOf('=');
    if (eq === -1) continue;
    const key = trimmed.slice(0, eq).trim();
    let value = trimmed.slice(eq + 1).trim();
    if ((value.startsWith('"') && value.endsWith('"')) || (value.startsWith("'") && value.endsWith("'"))) {
      value = value.slice(1, -1);
    }
    // Real environment variables (e.g. set by AWS) always win over .env
    if (process.env[key] === undefined) process.env[key] = value;
  }
  return true;
}

export function getConfig() {
  const repos = (process.env.GITHUB_REPOS || '')
    .split(',')
    .map((r) => r.trim())
    .filter(Boolean)
    .filter((r) => /^[\w.-]+\/[\w.-]+$/.test(r));

  return {
    port: Number(process.env.PORT || 3000),
    host: process.env.HOST || '0.0.0.0',
    githubToken: process.env.GITHUB_TOKEN || '',
    githubApiUrl: (process.env.GITHUB_API_URL || 'https://api.github.com').replace(/\/$/, ''),
    repos,
    dashboardTitle: process.env.DASHBOARD_TITLE || 'Graduation Project Progress: Multi-agentic LLM Robot mission planning system',
    projectStart: validDate(process.env.PROJECT_START_DATE),
    projectEnd: validDate(process.env.PROJECT_END_DATE),
    cacheTtlSeconds: Number(process.env.CACHE_TTL_SECONDS || 300),
    activityWeeks: Number(process.env.ACTIVITY_WEEKS || 12),
    dashboardUser: process.env.DASHBOARD_USER || 'admin',
    dashboardPassword: process.env.DASHBOARD_PASSWORD || '',
  };
}

function validDate(value) {
  if (!value) return null;
  const d = new Date(value);
  return Number.isNaN(d.getTime()) ? null : d.toISOString();
}
