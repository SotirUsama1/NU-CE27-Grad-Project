#!/usr/bin/env sh
# macOS / Linux: run "./start.sh" in this folder to start the dashboard.
# Press Ctrl+C to stop it.
cd "$(dirname "$0")" || exit 1

if ! command -v node >/dev/null 2>&1; then
  echo "Node.js is not installed. See README.md, step 1."
  exit 1
fi

if [ ! -d node_modules ]; then
  echo "Installing packages..."
  npm install --no-audit --no-fund
fi

# Open the browser a few seconds after the server starts
( sleep 3; (command -v open >/dev/null && open http://localhost:3000) || (command -v xdg-open >/dev/null && xdg-open http://localhost:3000) ) >/dev/null 2>&1 &

node server.js
