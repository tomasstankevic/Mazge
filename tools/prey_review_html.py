"""Generate compact HTML review of all prey-positive bursts.

Output: captures/prey_review/index.html
For each burst:
  - Folder name as title
  - Date+time (CET) parsed from meta.json epoch
  - 10 thumbnails in a row, with API-flagged frames highlighted
  - Click thumbnail to open full-size JPG
"""
import datetime
import html
import json
import os
import sys
import zoneinfo
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/prey_review")
OUT = ROOT / "index.html"
TZ = zoneinfo.ZoneInfo("Europe/Berlin")  # CET / CEST

bursts = []
for d in sorted(ROOT.iterdir()):
    if not d.is_dir():
        continue
    meta = d / "meta.json"
    if not meta.exists():
        continue
    try:
        m = json.loads(meta.read_text())
    except Exception:
        continue
    epoch = m.get("epoch")
    when = (datetime.datetime.fromtimestamp(epoch, tz=TZ).strftime("%Y-%m-%d %H:%M:%S %Z")
            if epoch else "(no epoch)")
    api = m.get("apiResults") or []
    files = sorted(d.iterdir())
    img_by_idx = {}
    for f in files:
        if f.suffix.lower() != ".jpg":
            continue
        # filename like f05.jpg or f05_0500ms.jpg
        n = f.stem
        if not n.startswith("f"):
            continue
        try:
            i = int(n[1:3])
        except ValueError:
            continue
        img_by_idx[i] = f.name
    bursts.append({
        "name": d.name,
        "when": when,
        "api": api,
        "img": img_by_idx,
        "result": m.get("apiResult"),
        "frames": m.get("frames"),
    })


def cell(burst, i):
    fname = burst["img"].get(i)
    flag = burst["api"][i] if i < len(burst["api"]) else -1
    if not fname:
        return f'<div class="thumb missing">f{i:02d}</div>'
    klass = "thumb"
    if flag == 1:
        klass += " prey"
    elif flag == 0:
        klass += " clear"
    elif flag == -1:
        klass += " skip"
    label = "PREY" if flag == 1 else ("OK" if flag == 0 else "—")
    src = f"{burst['name']}/{fname}"
    return (f'<a class="{klass}" href="{html.escape(src)}" target="_blank">'
            f'<img loading="lazy" src="{html.escape(src)}" alt="f{i:02d}">'
            f'<span class="lab">f{i:02d} {label}</span></a>')


head = """<!doctype html><html><head><meta charset="utf-8">
<title>Prey-positive burst review</title>
<style>
  body { background:#1a1a2e; color:#e0e0e0; font-family:system-ui,sans-serif;
         margin:0; padding:12px; font-size:13px; }
  h1 { color:#0f9; margin:0 0 4px; font-size:1.2em; }
  .summary { color:#888; margin:0 0 16px; }
  .burst { margin:0 0 18px; padding:8px; background:#222; border-radius:6px; }
  .burst h2 { font-size:0.95em; margin:0 0 6px; color:#8fa; font-weight:600; }
  .burst .meta { color:#999; font-size:0.85em; margin:0 0 6px; }
  .grid { display:flex; gap:4px; flex-wrap:wrap; }
  .thumb { display:block; width:96px; height:128px; position:relative;
           background:#000; border:2px solid #333; border-radius:4px;
           overflow:hidden; text-decoration:none; }
  .thumb img { width:128px; height:96px; object-fit:cover; transform:rotate(-90deg);
               transform-origin: 48px 48px; }
  .thumb .lab { position:absolute; bottom:0; left:0; right:0;
                background:rgba(0,0,0,0.7); color:#fff; font-size:10px;
                text-align:center; padding:1px 0; pointer-events:none; }
  .thumb.prey { border-color:#f44; box-shadow:0 0 6px #f44; }
  .thumb.prey .lab { color:#fbb; font-weight:bold; }
  .thumb.clear { border-color:#4a4; }
  .thumb.skip { border-color:#444; opacity:0.6; }
  .thumb.missing { color:#666; display:flex; align-items:center;
                   justify-content:center; font-size:11px; }
</style></head><body>"""

body = [head]
body.append('<h1>Prey-positive burst review</h1>')
body.append(f'<p class="summary">Total bursts: <b>{len(bursts)}</b>. '
            f'Red border = API flagged as prey, green = checked-clear, '
            f'gray = not checked (skipped due to early-exit).</p>')
for b in bursts:
    body.append('<div class="burst">')
    body.append(f'<h2>{html.escape(b["name"])}</h2>')
    body.append(f'<p class="meta">{html.escape(b["when"])} · '
                f'frames: {b["frames"]} · '
                f'overall: <b style="color:{"#f88" if b["result"]==1 else "#8f8"}">'
                f'{"PREY" if b["result"]==1 else "no prey" if b["result"]==0 else "?"}</b></p>')
    body.append('<div class="grid">')
    for i in range(10):
        body.append(cell(b, i))
    body.append('</div></div>')

body.append('</body></html>')
OUT.write_text("\n".join(body))
print(f"Wrote {OUT} ({len(bursts)} bursts)")
