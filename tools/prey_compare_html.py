"""Interactive 3-variant HTML report comparing prey detection results.

Variants:
  A - Original ESP32 live (from meta.json apiResults; only 1 frame typically)
  B - Re-run with current production pipeline (lossless DCT crop+rotate+drop-chroma)
  C - Optimized pipeline (full decode + crop right + rotate + bilinear resize + grayscale)

Features:
  - Three tabs (one per variant)
  - For each burst, 10 thumbnails with verdict overlay
  - Per-flagged-frame checkbox: untick = mark as false positive
  - Live stats footer: bursts confirmed (>=1 true positive), false-positive bursts,
                      per-frame hit-rate breakdown
"""
import datetime
import html
import json
import sys
import zoneinfo
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/prey_review")
OUT = ROOT / "compare.html"
TZ = zoneinfo.ZoneInfo("Europe/Berlin")

reproc_b = json.loads((ROOT / "reprocess.json").read_text())
reproc_c = json.loads((ROOT / "reprocess_optimized.json").read_text())

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
    name = d.name
    epoch = m.get("epoch")
    when = (datetime.datetime.fromtimestamp(epoch, tz=TZ).strftime("%Y-%m-%d %H:%M:%S %Z")
            if epoch else "(no epoch)")
    api_a = (m.get("apiResults") or []) + [-1] * 10
    api_a = api_a[:10]
    api_b = (reproc_b.get(name, {}).get("verdicts") or [None] * 10)
    api_c = (reproc_c.get(name, {}).get("verdicts") or [None] * 10)
    img_by_idx = {}
    for f in d.iterdir():
        if f.suffix.lower() != ".jpg":
            continue
        n = f.stem
        if not n.startswith("f"):
            continue
        try:
            i = int(n[1:3])
        except ValueError:
            continue
        img_by_idx[i] = f.name
    bursts.append({
        "name": name, "when": when, "img": img_by_idx,
        "A": api_a, "B": api_b, "C": api_c,
    })

bursts_json = json.dumps(bursts)


def render_thumbs(burst, variant_letter):
    cells = []
    api = burst[variant_letter]
    for i in range(10):
        fname = burst["img"].get(i)
        v = api[i] if i < len(api) else -1
        if v == 1:
            klass = "thumb prey"
            lab = "PREY"
        elif v == 0:
            klass = "thumb clear"
            lab = "OK"
        elif v is None or v == -1:
            klass = "thumb skip"
            lab = "—"
        else:
            klass = "thumb skip"
            lab = "?"
        chk_html = ""
        if v == 1:
            chk_html = (f'<input type="checkbox" class="confirm" checked '
                        f'data-burst="{html.escape(burst["name"])}" '
                        f'data-variant="{variant_letter}" data-frame="{i}">')
        if not fname:
            cells.append(f'<div class="thumb missing">f{i:02d}</div>')
            continue
        src = f"{burst['name']}/{fname}"
        cells.append(
            f'<div class="cell">'
            f'  <a class="{klass}" href="{html.escape(src)}" target="_blank">'
            f'    <img loading="lazy" src="{html.escape(src)}" alt="f{i:02d}">'
            f'    <span class="lab">f{i:02d} {lab}</span>'
            f'  </a>'
            f'  {chk_html}'
            f'</div>')
    return "\n".join(cells)


def render_variant(letter, label):
    out = [f'<section class="variant" id="var-{letter}" '
           f'style="display:{"block" if letter=="A" else "none"}">']
    out.append(f'<h2>{label}</h2>')
    for b in bursts:
        api = b[letter]
        hits = sum(1 for v in api if v == 1)
        out.append('<div class="burst">')
        out.append(f'<h3>{html.escape(b["name"])} '
                   f'<span class="meta">{html.escape(b["when"])}</span> '
                   f'<span class="hits" data-burst="{html.escape(b["name"])}" '
                   f'data-variant="{letter}">hits={hits}/10</span></h3>')
        out.append('<div class="grid">')
        out.append(render_thumbs(b, letter))
        out.append('</div></div>')
    out.append('</section>')
    return "\n".join(out)


HEAD = """<!doctype html><html><head><meta charset="utf-8">
<title>Prey detection 3-way comparison</title>
<style>
  body { background:#1a1a2e; color:#e0e0e0; font-family:system-ui,sans-serif;
         margin:0; padding:12px 12px 200px; font-size:13px; }
  h1 { color:#0f9; margin:0 0 4px; font-size:1.3em; }
  .nav { display:flex; gap:6px; margin:8px 0; }
  .nav button { background:#333; color:#eee; border:1px solid #555; padding:6px 14px;
                border-radius:4px; cursor:pointer; font-size:0.9em; }
  .nav button.active { background:#0f9; color:#000; font-weight:600; }
  .summary { color:#888; margin:0 0 12px; }
  .burst { margin:0 0 14px; padding:6px 8px; background:#222; border-radius:5px; }
  .burst h3 { font-size:0.9em; margin:0 0 5px; color:#8fa; font-weight:600; }
  .burst h3 .meta { color:#888; margin-left:6px; font-weight:400; font-size:0.9em; }
  .burst h3 .hits { float:right; color:#fbb; font-family:monospace; }
  .grid { display:flex; gap:4px; flex-wrap:wrap; }
  .cell { position:relative; }
  .cell .confirm { position:absolute; top:2px; right:2px; z-index:2;
                   width:18px; height:18px; cursor:pointer; }
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
  .thumb.skip { border-color:#444; opacity:0.5; }
  .thumb.missing { color:#666; display:flex; align-items:center;
                   justify-content:center; font-size:11px; }
  .footer { position:fixed; left:0; right:0; bottom:0; padding:10px 12px;
            background:#0a0a1a; border-top:2px solid #0f9; max-height:180px;
            overflow:auto; font-size:12px; }
  .footer h2 { color:#0f9; margin:0 0 6px; font-size:1em; }
  .stat-grid { display:grid; grid-template-columns:repeat(3, 1fr); gap:14px; }
  .stat-grid section { padding:0 6px; }
  .stat-grid h3 { margin:0 0 4px; color:#8af; font-size:0.95em; }
  .stat-grid table { font-family:monospace; font-size:11px; border-collapse:collapse; }
  .stat-grid td { padding:1px 6px; }
  .stat-grid td.bar { color:#0f9; }
  .legend { color:#777; font-size:11px; margin-top:4px; }
</style></head><body>"""

JS = """
<script>
const VARIANTS = ['A','B','C'];
function showTab(letter) {
  VARIANTS.forEach(v => {
    document.getElementById('var-'+v).style.display = (v===letter ? 'block' : 'none');
    document.getElementById('btn-'+v).classList.toggle('active', v===letter);
  });
}
function recompute() {
  // For each variant compute hits/burst respecting unticked checkboxes
  const stats = {};
  VARIANTS.forEach(v => stats[v] = {byBurst:{}, byFrame:{}, falsePosBursts:0,
                                     totalPreyBursts:0, totalFlagged:0,
                                     totalConfirmed:0});
  document.querySelectorAll('input.confirm').forEach(cb => {
    const b = cb.dataset.burst, v = cb.dataset.variant, f = parseInt(cb.dataset.frame);
    const s = stats[v];
    s.byBurst[b] = s.byBurst[b] || {flagged:0, confirmed:0};
    s.byBurst[b].flagged++;
    s.totalFlagged++;
    if (cb.checked) {
      s.byBurst[b].confirmed++;
      s.totalConfirmed++;
      s.byFrame[f] = (s.byFrame[f] || 0) + 1;
    }
  });
  // Update per-burst hit display
  document.querySelectorAll('.hits').forEach(h => {
    const b = h.dataset.burst, v = h.dataset.variant;
    const bb = stats[v].byBurst[b];
    if (bb) {
      h.textContent = `hits=${bb.confirmed}/10 (${bb.flagged} flagged)`;
      h.style.color = bb.confirmed > 0 ? '#fbb' : '#888';
    } else {
      h.textContent = 'hits=0/10';
      h.style.color = '#888';
    }
  });
  // Bursts with at least one confirmed = "true prey"; flagged bursts
  // with all unticked = "false positive bursts"
  VARIANTS.forEach(v => {
    const s = stats[v];
    s.totalPreyBursts = Object.values(s.byBurst).filter(b=>b.flagged>0).length;
    s.falsePosBursts = Object.values(s.byBurst).filter(b=>b.flagged>0 && b.confirmed===0).length;
    s.confirmedBursts = Object.values(s.byBurst).filter(b=>b.confirmed>0).length;
  });
  // Render footer
  const html = VARIANTS.map(v => {
    const s = stats[v];
    let frameRows = '';
    for (let i = 0; i < 10; i++) {
      const c = s.byFrame[i] || 0;
      const bar = '#'.repeat(c);
      frameRows += `<tr><td>f${String(i).padStart(2,'0')}</td>`+
                   `<td>${c}</td><td class="bar">${bar}</td></tr>`;
    }
    return `<section><h3>${v}</h3>`+
           `<div>flagged bursts: ${s.totalPreyBursts}</div>`+
           `<div>confirmed (>=1 true): <b style="color:#0f9">${s.confirmedBursts}</b></div>`+
           `<div>false-pos bursts: <b style="color:#f88">${s.falsePosBursts}</b></div>`+
           `<div>flagged frames: ${s.totalFlagged} confirmed: ${s.totalConfirmed}</div>`+
           `<table>${frameRows}</table></section>`;
  }).join('');
  document.getElementById('stats').innerHTML = html;
  // Persist tick state in localStorage
  const ticks = {};
  document.querySelectorAll('input.confirm').forEach(cb => {
    ticks[`${cb.dataset.variant}|${cb.dataset.burst}|${cb.dataset.frame}`] = cb.checked;
  });
  localStorage.setItem('preyTicks', JSON.stringify(ticks));
}
function loadTicks() {
  try {
    const ticks = JSON.parse(localStorage.getItem('preyTicks') || '{}');
    document.querySelectorAll('input.confirm').forEach(cb => {
      const k = `${cb.dataset.variant}|${cb.dataset.burst}|${cb.dataset.frame}`;
      if (k in ticks) cb.checked = ticks[k];
    });
  } catch (e) {}
}
document.addEventListener('DOMContentLoaded', () => {
  loadTicks();
  document.querySelectorAll('input.confirm').forEach(cb => {
    cb.addEventListener('change', recompute);
  });
  recompute();
});
</script>"""

body = [HEAD, '<h1>Prey detection — 3-variant comparison</h1>']
body.append(f'<p class="summary">{len(bursts)} prey-positive bursts. '
            f'Click thumbnails to view full size. Untick checkboxes on flagged '
            f'frames you consider false positives — stats update live and persist '
            f'in your browser.</p>')
body.append('<div class="nav">')
body.append('<button id="btn-A" class="active" onclick="showTab(\'A\')">A: ESP32 live (meta)</button>')
body.append('<button id="btn-B" onclick="showTab(\'B\')">B: Lossless DCT (current production)</button>')
body.append('<button id="btn-C" onclick="showTab(\'C\')">C: Optimized (decode+resize)</button>')
body.append('</div>')
body.append(render_variant("A", "A — ESP32 live results from meta.json (apiResults)"))
body.append(render_variant("B", "B — Re-run with lossless DCT crop+rotate+drop-chroma (current production)"))
body.append(render_variant("C", "C — Optimized: PIL decode + crop right + 90° CCW + LANCZOS resize 384 + grayscale q=90"))
body.append('<div class="footer"><h2>Stats (after your false-positive ticks)</h2>'
            '<div class="stat-grid" id="stats"></div>'
            '<div class="legend">Untick a flagged frame to mark it as false positive. '
            'Bursts with ≥1 confirmed = true prey detection. State saved per browser.</div>'
            '</div>')
body.append(JS)
body.append('</body></html>')
OUT.write_text("\n".join(body))
print(f"Wrote {OUT} ({len(bursts)} bursts × 3 variants)")
