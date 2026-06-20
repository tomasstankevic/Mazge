# Mazge \u2192 Home Assistant configuration

This folder holds the Home Assistant configuration that is mounted into the
HA container running on this Mac.

## Layout

- `configuration.yaml` \u2014 the main HA config. Reads MQTT creds from `secrets.yaml`.
- `secrets.yaml` \u2014 NOT committed. Generated from `secrets.yaml.example`.
- `dashboards/mazge.yaml` \u2014 the Lovelace dashboard for the cat door (status, latest frame, recent bursts gallery).
- `secrets.yaml.example` \u2014 template; copy to `secrets.yaml` and fill values.

## Container setup (Colima + Docker)

```bash
# One-time: start the Linux VM that hosts containers (4GB RAM, 2 CPU).
colima start --cpu 2 --memory 4 --disk 20

# Pull and run Home Assistant. The HA container talks to:
#   - the Mazge HTTP server on host:8080
#   - the Mosquitto broker on host:1883
# Colima exposes the host via `host.lima.internal`.
docker run -d \
  --name homeassistant \
  --restart unless-stopped \
  -p 8123:8123 \
  -v $HOME/Mazge/ops/home-assistant:/config \
  -e TZ=Europe/Prague \
  ghcr.io/home-assistant/home-assistant:stable
```

Open http://mazge.local:8123 in a browser, create the owner account, then
restart HA so `configuration.yaml` is loaded.

## Remote access via Tailscale

The Mac is already on Tailscale (`mazge=100.85.62.100`). On your phone:

1. Install the Tailscale app and sign in with the same account.
2. Install the Home Assistant Companion app.
3. Add server: `http://100.85.62.100:8123` (or the MagicDNS name `http://mazge:8123`).

When you leave the LAN, Tailscale keeps the same URL reachable over the VPN.

## What the dashboard shows

- **Mazge server status** (online / offline, uptime, backend) \u2014 via MQTT LWT.
- **Latest frame** \u2014 live camera entity refreshing every 5 s from `/v1/latest_frame.jpg`.
- **Last decision** \u2014 prey score, cat ID, severity, door action.
- **Recent bursts** \u2014 markdown card listing the last 10 bursts with thumbnails (`/v1/bursts/recent`).

Door control is a stub for now: a button publishes to `mazge/door/cmd`. The
ESP32 firmware does not subscribe to MQTT yet; that lands when we do the OTA
flash. Until then the button is a no-op placeholder so the dashboard layout
is final.
