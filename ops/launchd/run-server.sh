#!/bin/zsh
# Wrapper for launchd that loads env from ~/.config/mazge/server.env (untracked)
# and starts the Mazge server. The plist invokes this so secrets do not live in
# the tracked plist.
set -e
ENV_FILE="${HOME}/.config/mazge/server.env"
if [[ -f "$ENV_FILE" ]]; then
  set -a
  source "$ENV_FILE"
  set +a
fi
exec /Users/tomas/Mazge/.venv-server/bin/uvicorn server.app:app \
  --host 0.0.0.0 \
  --port "${MAZGE_PORT:-8080}"
