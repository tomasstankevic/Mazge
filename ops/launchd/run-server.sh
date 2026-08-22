#!/bin/zsh
# Wrapper for launchd that loads env from ~/.config/mazge/server.env (untracked)
# and starts the Mazge server. The plist invokes this so secrets do not live in
# the tracked plist.
set -e
ENV_FILE="${HOME}/.config/mazge/server.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: $ENV_FILE not found. Server requires critical config like MAZGE_ESP_HOST." >&2
  exit 1
fi

set -a
source "$ENV_FILE"
set +a

# Validate critical config exists
if [[ -z "$MAZGE_ESP_HOST" ]]; then
  echo "ERROR: MAZGE_ESP_HOST not set in $ENV_FILE" >&2
  exit 1
fi

echo "Starting Mazge server with ESP_HOST=$MAZGE_ESP_HOST"
exec /Users/tomas/Mazge/.venv-server/bin/uvicorn server.app:app \
  --host 0.0.0.0 \
  --port "${MAZGE_PORT:-8080}"
