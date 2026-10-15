# GitHub Progress Dashboard

A web dashboard that shows live GitHub progress for the graduation project
**Multi-agentic LLM Robot mission planning system**
([SotirUsama1/NU-CE27-Grad-Project](https://github.com/SotirUsama1/NU-CE27-Grad-Project)):

- **Hero header** with the project title, days in development, team size, an
  overall progress ring and a project timeline (deadline countdown if set)
- **Commits on every branch** (deduplicated), not only the default branch,
  plus commits this week and a **commit-velocity chart** (all branches / main)
- **Development rhythm** heatmap of daily commits over the last 26 weeks
- **Feature tracks**: each branch with its status (active, merged, idle),
  last commit and how far it is ahead of / behind `main`
- **CI / CD pipelines**: GitHub Actions pass rate, workflow status and recent runs
- **Pull requests & issues** (open, merged, closed) and **milestones** with % complete
- **Tech stack** (languages) and the latest commits timeline
- Light and dark themes (follows the system, toggle in the top bar)
- An **"All repositories"** view when you track several repos, and a
  **Deployment** panel for AWS hosting

The GitHub token stays on the server and is never sent to the browser.

## Run it on a new laptop

You need three things: **Node.js**, the **project folder**, and a **GitHub token**.

### Step 1: Install Node.js (once per laptop)

Node.js 18 or newer is required. Pick your system and run the commands in a terminal.

**Windows** (PowerShell or Command Prompt):

```powershell
winget install -e --id OpenJS.NodeJS.LTS
```

Then **close the terminal and open a new one**, so it picks up Node.js.
(No winget? Download the "LTS" installer from https://nodejs.org and click through it.)

**macOS** (with [Homebrew](https://brew.sh)):

```bash
brew install node
```

(Or download the "LTS" installer from https://nodejs.org.)

**Linux** (Ubuntu / Debian):

```bash
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs
```

**Check it worked** (both should print a version number):

```bash
node --version
npm --version
```

### Step 2: Copy the project folder

Copy the whole `github-dashboard` folder to the new laptop (USB, zip, cloud drive...).
You can **skip the `node_modules` folder**; it is re-created in step 4.

### Step 3: Create the settings file (`.env`)

If you copied your `.env` file along with the folder, skip this step.
Otherwise, open a terminal **inside the `github-dashboard` folder** and run:

```powershell
# Windows
copy .env.example .env
```

```bash
# macOS / Linux
cp .env.example .env
```

Open `.env` in any text editor and paste your GitHub token after `GITHUB_TOKEN=`
(no spaces, no quotes), then save:

```
GITHUB_TOKEN=github_pat_xxxxxxxxxxxxxxxx
```

How to get a token: see [Getting a GitHub token](#getting-a-github-token) below.
The dashboard also works without a token, but GitHub then allows only 60 requests
per hour, which runs out after 2-3 refreshes.

### Step 4: Install the required libraries (once)

In a terminal inside the `github-dashboard` folder:

```bash
npm install
```

### Step 5: Run it

- **Windows:** double-click **`start.bat`**
- **macOS / Linux:** in a terminal in this folder, run `./start.sh`
  (the first time, allow it with `chmod +x start.sh`)
- **Any system:** `npm start`

The dashboard opens at **http://localhost:3000**.
To stop it, close the window (Windows) or press **Ctrl + C** in the terminal.

`start.bat` and `start.sh` also run `npm install` automatically if step 4 was
skipped, so after step 1 and step 3 a double-click is enough.

### Getting a GitHub token

1. Sign in at **github.com** with your own account.
2. Profile picture (top right) → **Settings** → **Developer settings** (bottom of the
   left menu) → **Personal access tokens** → **Fine-grained tokens** → **Generate new token**.
3. **Token name:** `dashboard` · **Expiration:** e.g. 1 year · **Repository access:** **Public repositories**.
4. Click **Generate token** and copy it (starts with `github_pat_`; it is shown only once).

Alternative: **Tokens (classic)** → *Generate new token (classic)*, tick **no** scopes,
generate (starts with `ghp_`). Either kind goes into `GITHUB_TOKEN` in `.env`.

Keep the token private: don't share `.env` publicly or upload it to GitHub.

### Troubleshooting

| Problem | Fix |
|---|---|
| `node` / `npm` is "not recognized" | Close and reopen the terminal after installing Node.js. If it persists, restart the laptop. |
| Numbers show **0** or "rate limit reached" | Add a `GITHUB_TOKEN` to `.env` and restart the dashboard. |
| "Port 3000 is already in use" (`EADDRINUSE`) | The dashboard is already running in another window, or change `PORT=3001` in `.env`. |
| Changes to `.env` have no effect | Stop the dashboard and start it again; `.env` is read at startup. |
| `./start.sh: Permission denied` | Run `chmod +x start.sh` once. |

## Settings (`.env`)

| Variable | What it does |
|---|---|
| `GITHUB_TOKEN` | Read-only GitHub token (required for private repos) |
| `GITHUB_REPOS` | Comma-separated `owner/repo` list |
| `DASHBOARD_TITLE` | Title shown at the top (text before `:` becomes the small label above it) |
| `PROJECT_START_DATE` | Project kick-off, `YYYY-MM-DD` (defaults to the repo's creation date) |
| `PROJECT_END_DATE` | Deadline, `YYYY-MM-DD`: shows a countdown and % of schedule used |
| `PORT` | Local port (default `3000`) |
| `CACHE_TTL_SECONDS` | How often data is refreshed from GitHub (default 300; at least 1800 without a token) |
| `ACTIVITY_WEEKS` | Weeks shown in the activity chart (default 12) |
| `DASHBOARD_PASSWORD` | Turns on a login prompt (user = `DASHBOARD_USER`) |
| `GITHUB_API_URL` | Only for GitHub Enterprise Server |
| `DEPLOY_TARGET`, `AWS_REGION`, `AWS_SECRET_NAME`, `PUBLIC_URL` | AWS section, see below |

## Hosting on AWS

See **[docs/AWS.md](docs/AWS.md)**. Short version: build the Docker image, push
it to Amazon ECR, and create an **ECS Express Mode** service (port `8080`,
health check `/health`) with the same environment variables. ECS gives you a
public link. Everything AWS-specific lives in:

- `src/aws.js`: reads the token from AWS Secrets Manager and reports status
- `Dockerfile`: the container image
- `.env.example`: the "AWS CONNECTION SECTION" variables
- the **AWS connection** panel at the bottom of the dashboard

## How the numbers are counted

- **Commits:** unique commits across all branches (each commit counted once).
  The "on main line" number is the default branch only.
- **Feature tracks:** "ahead" = commits not yet in `main`; "merged" = nothing
  left to merge; "active" = a commit in the last 14 days.
- **Progress ring:** milestone completion if milestones exist; otherwise
  % of the schedule used (when `PROJECT_END_DATE` is set); otherwise the share
  of feature branches fully merged into `main`.
- **Merged PRs / closed issues:** all-time totals from GitHub search.
- **Commits per week:** all branches (or `main` only via the tab), weeks start Monday (UTC).
- Data is cached for `CACHE_TTL_SECONDS`; the **Refresh** button bypasses the cache
  (at most every 30 s with a token, every 15 min without one).
- One full load uses about 25 GitHub API requests. Without a token GitHub allows
  60 per hour, so a read-only token is strongly recommended.

## Privacy

The dashboard only **reads** public data; it never writes to the repository
(no commits, comments, stars or forks). Repository owners cannot see who reads
their repository through the API: GitHub's traffic page only shows anonymous
totals. Without a token, requests are anonymous. With a token, GitHub itself
knows which account made the requests, but the repo owner does not.

## Project layout

```
server.js          local web server + API (/api/config, /api/summary, /api/overview, /health)
src/github.js      GitHub API calls and number crunching
src/config.js      reads .env
src/aws.js         AWS connection section
public/            the dashboard page (HTML/CSS/JS, no build step)
start.bat          one-click start on Windows
start.sh           one-command start on macOS / Linux
docs/AWS.md        AWS deployment guide
Dockerfile         container for AWS
```
