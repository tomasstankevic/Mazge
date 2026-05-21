"""Local labeling UI for prey bursts.

Three orthogonal axes per burst, applied to all 10 frames:
  - prey:      yes / no / unclear
  - direction: entering / exiting / unclear
  - subject:   cat / human / other / unclear   (overrides YOLO auto-label)

Labels are appended to dataset/labels.jsonl using sources:
  source = "human:<user>:burst_prey"      label = 0|1
  source = "human:<user>:burst_direction" label = 0 (entering) | 1 (exiting)
  source = "human:<user>:burst_subject"   label = 0 empty | 1 cat | 2 human | 3 other
notes field carries the "unclear" status so we can distinguish "I labelled
it unclear" from "no label yet".

Usage:
  uv run python tools/label_bursts.py
  # then open http://127.0.0.1:8765/

  # Filter what shows in the queue:
  uv run python tools/label_bursts.py --filter prey-positive  # default
  uv run python tools/label_bursts.py --filter all
  uv run python tools/label_bursts.py --filter unlabelled
  uv run python tools/label_bursts.py --filter disagreement   # fw vs full
  uv run python tools/label_bursts.py --filter human          # YOLO saw human
  uv run python tools/label_bursts.py --filter other          # YOLO saw bird/dog

Keyboard:
  y / n / u   prey: yes / no / unclear
  e / x / U   direction: entering / exiting / unclear (capital U)
  c / h / o   subject: cat / human / other  (E for empty, S for unclear)
  enter       save and advance
  ←  →        prev / next burst
  1..9 0      jump to frame f0..f9
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import socketserver
import urllib.parse
from http import server
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
LABELS = REPO / "dataset" / "labels.jsonl"
USER = os.environ.get("USER", "tomas")

# In-memory snapshot of bursts to label, ordered. Built once at startup.
BURSTS: list[dict] = []

# Cached label index: image_id -> {prey, direction, subject, yolo_subject, yolo_notes}
# Built once at startup, then mutated when /api/label saves new records.
LABEL_INDEX: dict[str, dict] = {}


def rebuild_label_index() -> None:
    """Read labels.jsonl once and populate LABEL_INDEX. O(N) instead of O(N*M)."""
    LABEL_INDEX.clear()
    if not LABELS.exists():
        return
    with LABELS.open() as f:
        for line in f:
            try:
                rec = json.loads(line)
            except Exception:
                continue
            image_id = rec.get("image_id", "")
            if not image_id:
                continue
            entry = LABEL_INDEX.setdefault(image_id, {
                "prey": None, "direction": None, "subject": None,
                "yolo_subject": None, "yolo_notes": None,
            })
            src = rec.get("source", "")
            if src.startswith("human:") and src.endswith(":burst_prey"):
                entry["prey"] = {"label": rec.get("label"), "notes": rec.get("notes")}
            elif src.startswith("human:") and src.endswith(":burst_direction"):
                entry["direction"] = {"label": rec.get("label"), "notes": rec.get("notes")}
            elif src.startswith("human:") and src.endswith(":burst_subject"):
                entry["subject"] = {"label": rec.get("label"), "notes": rec.get("notes")}
            elif src == "model:yolo11n_subject_v1":
                entry["yolo_subject"] = rec.get("label")
                entry["yolo_notes"] = rec.get("notes")


def burst_meta(folder: Path) -> dict | None:
    p = folder / "meta.json"
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None


def existing_labels(image_id: str) -> dict:
    """Return the latest human burst-level labels + YOLO subject for image_id.
    O(1) lookup using the cached LABEL_INDEX."""
    return LABEL_INDEX.get(image_id, {
        "prey": None, "direction": None, "subject": None,
        "yolo_subject": None, "yolo_notes": None,
    })


def build_queue(filter_mode: str, since: str | None = None,
                until: str | None = None) -> list[dict]:
    """Return list of burst dicts to label, in priority order.

    `since` / `until` are inclusive 8-digit date prefixes (YYYYMMDD) matched
    against the burst folder name. Folders whose name does not start with an
    8-digit date are skipped when either bound is set.
    """
    items = []
    for d in sorted(SD.iterdir()):
        if not d.is_dir():
            continue
        if since or until:
            prefix = d.name[:8]
            if not (len(prefix) == 8 and prefix.isdigit()):
                continue
            if since and prefix < since:
                continue
            if until and prefix > until:
                continue
        meta = burst_meta(d)
        if not meta:
            continue
        # Need at least f07 (or whichever frame is closest to it) on disk
        images = meta.get("images", [])
        if not images:
            continue
        # Find a representative frame (closest index to 7)
        avail = [(i, img.get("f"))
                 for i, img in enumerate(images)
                 if (d / img.get("f", "")).exists()]
        if not avail:
            continue
        rep_idx, rep_name = min(avail, key=lambda t: abs(t[0] - 7))
        api_results = meta.get("apiResults", [])
        fw_label = 1 if meta.get("apiResult") == 1 else 0
        # Read full_analysis.json if present
        fa = d / "full_analysis.json"
        full_label = None
        if fa.exists():
            try:
                fad = json.loads(fa.read_text())
                fp = fad.get("full_prey_count", 0)
                full_label = 1 if fp >= 2 else 0
            except Exception:
                pass

        # Priority: prey-positive (any source) > disagreement > unlabelled
        is_prey = (fw_label == 1) or (full_label == 1)
        is_disagreement = full_label is not None and full_label != fw_label

        # Aggregate YOLO subject across all frames in this burst.
        # Read labels.jsonl is expensive per-image — do it once per burst here
        # by scanning labels for this burst's frames. (build_queue is one-shot.)
        yolo_per_frame: list[int | None] = []
        for img in images:
            name = img.get("f")
            if not name:
                continue
            iid = f"captures/sd/{d.name}/{name}"
            ex = existing_labels(iid)
            yolo_per_frame.append(ex["yolo_subject"])
        yolo_counts = {0: 0, 1: 0, 2: 0, 3: 0}
        for v in yolo_per_frame:
            if v in yolo_counts:
                yolo_counts[v] += 1
        # Burst-level YOLO subject = max-vote (excluding empty unless all empty)
        nonempty = {k: v for k, v in yolo_counts.items() if k != 0}
        if nonempty and max(nonempty.values()) > 0:
            burst_yolo = max(nonempty, key=lambda k: nonempty[k])
        else:
            burst_yolo = 0  # all empty

        items.append({
            "burst_id": d.name,
            "rep_idx": rep_idx,
            "rep_name": rep_name,
            "frames": [{"idx": i,
                        "name": img.get("f"),
                        "exists": (d / img.get("f", "")).exists(),
                        "dist": img.get("dist", -1),
                        "ms": img.get("offsetMs", img.get("ms")),
                        "yolo": yolo_per_frame[i] if i < len(yolo_per_frame) else None}
                       for i, img in enumerate(images)],
            "fw_label": fw_label,
            "full_label": full_label,
            "fw_prey_count": sum(1 for r in api_results if r == 1),
            "fw_sent": sum(1 for r in api_results if r != -1),
            "is_prey": is_prey,
            "is_disagreement": is_disagreement,
            "burst_yolo": burst_yolo,        # 0=empty 1=cat 2=human 3=other
            "yolo_counts": yolo_counts,
        })

    if filter_mode == "prey-positive":
        items = [x for x in items if x["is_prey"]]
    elif filter_mode == "disagreement":
        items = [x for x in items if x["is_disagreement"]]
    elif filter_mode == "human":
        items = [x for x in items if x["burst_yolo"] == 2 or x["yolo_counts"][2] >= 2]
    elif filter_mode == "other":
        items = [x for x in items if x["burst_yolo"] == 3 or x["yolo_counts"][3] >= 1]
    elif filter_mode == "unlabelled":
        # Keep items whose representative frame doesn't have a human label
        # for both prey AND direction.
        kept = []
        for x in items:
            rep_id = f"captures/sd/{x['burst_id']}/{x['rep_name']}"
            ex = existing_labels(rep_id)
            if ex["prey"] is None or ex["direction"] is None or ex["subject"] is None:
                kept.append(x)
        items = kept
    elif filter_mode == "all":
        pass
    else:
        raise SystemExit(f"unknown --filter {filter_mode}")

    # Sort: prey-positive first, then disagreements, then by date desc
    items.sort(
        key=lambda x: (
            0 if x["is_prey"] else (1 if x["is_disagreement"] else 2),
            -int(x["burst_id"][:8]) if x["burst_id"][:8].isdigit() else 0,
        )
    )
    return items


HTML = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Burst labeller</title>
<style>
  :root { color-scheme: dark; }
  body { font: 14px system-ui, sans-serif; background:#181818; color:#ddd; margin:0; padding:12px; }
  header { display:flex; justify-content:space-between; align-items:center; gap:12px; margin-bottom:8px; }
  header .meta { font-size:13px; color:#aaa; }
  .progress { color:#9c9; font-weight:600; }
  .row { display:flex; gap:16px; margin-bottom:12px; }
  .main-img { background:#000; border-radius:6px; max-width:640px; max-height:480px; }
  .meta-table { font-size:13px; min-width:280px; }
  .meta-table td { padding:2px 8px; }
  .meta-table td:first-child { color:#999; }
  .strip { display:flex; gap:4px; padding:6px; background:#222; border-radius:6px; overflow-x:auto; }
  .strip img { height:80px; cursor:pointer; border:2px solid transparent; border-radius:3px; opacity:0.6; }
  .strip img.active { border-color:#5af; opacity:1.0; }
  .strip .ph { width:120px; height:80px; background:#111; border:2px solid transparent; border-radius:3px;
               display:flex; align-items:center; justify-content:center; color:#666; font-size:11px; }
  .controls { display:flex; flex-direction:column; gap:12px; padding:12px; background:#222; border-radius:6px; min-width:320px; }
  .group { display:flex; flex-direction:column; gap:6px; }
  .group .label { font-size:12px; color:#aaa; text-transform:uppercase; letter-spacing:.05em; }
  .btnrow { display:flex; gap:6px; }
  .btn { padding:8px 14px; background:#333; border:1px solid #444; color:#eee; border-radius:4px; cursor:pointer; font-size:14px; }
  .btn:hover { background:#444; }
  .btn.sel { background:#395; border-color:#5b7; color:#fff; font-weight:600; }
  .btn.sel.no { background:#a55; border-color:#c77; }
  .btn.sel.unclear { background:#88a; border-color:#aac; }
  .btn.sel.exit { background:#a83; border-color:#cb5; }
  .nav { display:flex; gap:8px; margin-top:6px; }
  .nav .btn { flex:1; }
  .saved { color:#9c9; font-size:12px; min-height:14px; }
  .existing { font-size:12px; color:#aac; padding:6px; background:#222; border-radius:4px; }
  .existing b { color:#ddd; }
  textarea { width:100%; box-sizing:border-box; background:#111; color:#ddd; border:1px solid #444; border-radius:4px; padding:6px; font-family:inherit; resize:vertical; min-height:50px; }
  kbd { background:#333; padding:1px 5px; border:1px solid #555; border-bottom-width:2px; border-radius:3px; font-size:11px; }
</style></head><body>
<header>
  <div><strong>Burst labeller</strong> &mdash; <span id="filter"></span></div>
  <div class="meta">User: <b id="user"></b></div>
  <div class="progress" id="progress"></div>
</header>

<div class="row">
  <div>
    <img id="mainImg" class="main-img">
    <div class="strip" id="strip"></div>
  </div>
  <div class="controls">
    <div class="meta">
      <table class="meta-table">
        <tr><td>burst</td><td><b id="burstId">&mdash;</b></td></tr>
        <tr><td>frame</td><td><b id="frameInfo">&mdash;</b></td></tr>
        <tr><td>distance</td><td id="distInfo">&mdash;</td></tr>
        <tr><td>API verdict</td><td id="apiInfo">&mdash;</td></tr>
      </table>
    </div>

    <div class="existing" id="existing">No prior human label</div>
    <div class="existing" id="yoloInfo" style="color:#bcb">YOLO: —</div>

    <div class="group">
      <div class="label">Prey <span style="color:#666">(y/n/u)</span></div>
      <div class="btnrow">
        <button class="btn" data-prey="1" id="btnPreyYes">Yes</button>
        <button class="btn no" data-prey="0" id="btnPreyNo">No</button>
        <button class="btn unclear" data-prey="u" id="btnPreyU">Unclear</button>
      </div>
    </div>

    <div class="group">
      <div class="label">Direction <span style="color:#666">(e/x/U)</span></div>
      <div class="btnrow">
        <button class="btn" data-dir="0" id="btnDirEnter">Entering</button>
        <button class="btn exit" data-dir="1" id="btnDirExit">Exiting</button>
        <button class="btn unclear" data-dir="u" id="btnDirU">Unclear</button>
      </div>
    </div>

    <div class="group">
      <div class="label">Subject <span style="color:#666">(c/h/o/E/S)</span></div>
      <div class="btnrow">
        <button class="btn" data-subj="1" id="btnSubjCat">Cat</button>
        <button class="btn no" data-subj="2" id="btnSubjHuman">Human</button>
        <button class="btn exit" data-subj="3" id="btnSubjOther">Other</button>
        <button class="btn unclear" data-subj="0" id="btnSubjEmpty">Empty</button>
        <button class="btn unclear" data-subj="u" id="btnSubjU">?</button>
      </div>
    </div>

    <div class="group">
      <div class="label">Notes (optional)</div>
      <textarea id="notes" placeholder="e.g. mouse, very fast, partial occlusion"></textarea>
    </div>

    <div class="saved" id="saved">&nbsp;</div>

    <div class="nav">
      <button class="btn" id="btnPrev">&larr; Prev</button>
      <button class="btn" id="btnSave">Save &amp; Next (Enter)</button>
      <button class="btn" id="btnSkip">Skip</button>
    </div>
  </div>
</div>

<script>
const state = { queue: [], idx: 0, current: null, frame: 7, prey: null, direction: null, subject: null };

async function fetchJSON(url, opts) {
  const r = await fetch(url, opts);
  if (!r.ok) throw new Error(url + " => " + r.status);
  return r.json();
}

function renderStrip() {
  const el = document.getElementById("strip");
  el.innerHTML = "";
  state.current.frames.forEach(f => {
    if (!f.exists) {
      const ph = document.createElement("div");
      ph.className = "ph"; ph.textContent = "f" + f.idx;
      el.appendChild(ph);
      return;
    }
    const img = document.createElement("img");
    img.src = "/image/captures/sd/" + state.current.burst_id + "/" + f.name;
    img.title = "f" + f.idx + " (" + f.ms + "ms, dist=" + f.dist + ")";
    img.className = (f.idx === state.frame) ? "active" : "";
    img.onclick = () => { state.frame = f.idx; render(); };
    el.appendChild(img);
  });
}

function render() {
  const c = state.current;
  if (!c) return;
  document.getElementById("burstId").textContent = c.burst_id;
  document.getElementById("progress").textContent = (state.idx + 1) + " / " + state.queue.length;
  // Find frame info
  const f = c.frames[state.frame];
  if (f && f.exists) {
    document.getElementById("mainImg").src = "/image/captures/sd/" + c.burst_id + "/" + f.name;
    document.getElementById("frameInfo").textContent = "f" + f.idx + "  (" + f.ms + " ms)";
    document.getElementById("distInfo").textContent = (f.dist >= 0 ? f.dist + " mm" : "n/a");
  } else {
    // Fallback to representative frame
    const rep = c.frames[c.rep_idx];
    document.getElementById("mainImg").src = "/image/captures/sd/" + c.burst_id + "/" + rep.name;
    document.getElementById("frameInfo").textContent = "f" + rep.idx + " (rep)";
    document.getElementById("distInfo").textContent = (rep.dist >= 0 ? rep.dist + " mm" : "n/a");
  }
  let api = "fw=" + (c.fw_label === 1 ? "PREY" : "no") + " (" + c.fw_prey_count + "/" + c.fw_sent + ")";
  if (c.full_label !== null) api += "; full=" + (c.full_label === 1 ? "PREY" : "no");
  document.getElementById("apiInfo").textContent = api;
  document.getElementById("apiInfo").style.color = (c.is_disagreement ? "#fb6" : "#ddd");

  // Existing human labels
  const ex = c.existing || {prey:null, direction:null, subject:null, yolo_subject:null, yolo_notes:null};
  const SUBJ = {0:"empty", 1:"cat", 2:"human", 3:"other"};
  const parts = [];
  if (ex.prey)      parts.push("prey=" + (ex.prey.label === 1 ? "yes" : ex.prey.label === 0 ? "no" : "unclear"));
  if (ex.direction) parts.push("dir="  + (ex.direction.label === 1 ? "exiting" : ex.direction.label === 0 ? "entering" : "unclear"));
  if (ex.subject)   parts.push("subj=" + (SUBJ[ex.subject.label] || "unclear"));
  document.getElementById("existing").innerHTML = parts.length ? "Prior human label: <b>" + parts.join(", ") + "</b>" : "No prior human label";

  // YOLO hint for the current frame + burst-level (reuse `f` declared above)
  let yoloLine = "YOLO burst: <b>" + SUBJ[c.burst_yolo] + "</b>";
  yoloLine += " (counts: cat=" + c.yolo_counts[1] + " human=" + c.yolo_counts[2] + " other=" + c.yolo_counts[3] + " empty=" + c.yolo_counts[0] + ")";
  if (f && f.yolo !== null && f.yolo !== undefined) {
    yoloLine += " — this frame: <b>" + SUBJ[f.yolo] + "</b>";
  }
  if (ex.yolo_notes) yoloLine += " — " + ex.yolo_notes;
  document.getElementById("yoloInfo").innerHTML = yoloLine;

  // Selection state
  for (const id of ["btnPreyYes","btnPreyNo","btnPreyU","btnDirEnter","btnDirExit","btnDirU",
                    "btnSubjCat","btnSubjHuman","btnSubjOther","btnSubjEmpty","btnSubjU"]) {
    document.getElementById(id).classList.remove("sel");
  }
  if (state.prey === 1) document.getElementById("btnPreyYes").classList.add("sel");
  if (state.prey === 0) document.getElementById("btnPreyNo").classList.add("sel");
  if (state.prey === "u") document.getElementById("btnPreyU").classList.add("sel");
  if (state.direction === 0) document.getElementById("btnDirEnter").classList.add("sel");
  if (state.direction === 1) document.getElementById("btnDirExit").classList.add("sel");
  if (state.direction === "u") document.getElementById("btnDirU").classList.add("sel");
  if (state.subject === 1) document.getElementById("btnSubjCat").classList.add("sel");
  if (state.subject === 2) document.getElementById("btnSubjHuman").classList.add("sel");
  if (state.subject === 3) document.getElementById("btnSubjOther").classList.add("sel");
  if (state.subject === 0) document.getElementById("btnSubjEmpty").classList.add("sel");
  if (state.subject === "u") document.getElementById("btnSubjU").classList.add("sel");

  renderStrip();
}

function selectPrey(v)      { state.prey = v;      render(); }
function selectDirection(v) { state.direction = v; render(); }
function selectSubject(v)   { state.subject = v;   render(); }

async function save() {
  if (state.prey === null && state.direction === null && state.subject === null) {
    document.getElementById("saved").textContent = "Nothing to save.";
    return;
  }
  const notes = document.getElementById("notes").value.trim();
  const res = await fetchJSON("/api/label", {
    method: "POST",
    headers: {"Content-Type":"application/json"},
    body: JSON.stringify({
      burst_id: state.current.burst_id,
      prey: state.prey,
      direction: state.direction,
      subject: state.subject,
      notes: notes || null,
    }),
  });
  document.getElementById("saved").textContent = "Saved " + res.added + " label record(s).";
  state.current.existing = res.existing;
  await advance(+1);
}

async function advance(d) {
  state.idx = Math.max(0, Math.min(state.queue.length - 1, state.idx + d));
  await loadIdx(state.idx);
}

async function loadIdx(i) {
  state.current = state.queue[i];
  state.frame = state.current.rep_idx;
  state.prey = state.current.existing?.prey?.label ?? null;
  state.direction = state.current.existing?.direction?.label ?? null;
  state.subject = state.current.existing?.subject?.label ?? null;
  document.getElementById("notes").value = "";
  document.getElementById("saved").textContent = "\u00a0";
  render();
}

window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "TEXTAREA") return;
  if (e.key === "y") selectPrey(1);
  else if (e.key === "n") selectPrey(0);
  else if (e.key === "u") selectPrey("u");
  else if (e.key === "e") selectDirection(0);
  else if (e.key === "x") selectDirection(1);
  else if (e.key === "U") selectDirection("u");
  else if (e.key === "c") selectSubject(1);
  else if (e.key === "h") selectSubject(2);
  else if (e.key === "o") selectSubject(3);
  else if (e.key === "E") selectSubject(0);
  else if (e.key === "S") selectSubject("u");
  else if (e.key === "Enter") save();
  else if (e.key === "ArrowRight") advance(+1);
  else if (e.key === "ArrowLeft")  advance(-1);
  else if (e.key >= "0" && e.key <= "9") {
    const i = parseInt(e.key); state.frame = i; render();
  }
});

document.getElementById("btnPreyYes").onclick = () => selectPrey(1);
document.getElementById("btnPreyNo").onclick  = () => selectPrey(0);
document.getElementById("btnPreyU").onclick   = () => selectPrey("u");
document.getElementById("btnDirEnter").onclick= () => selectDirection(0);
document.getElementById("btnDirExit").onclick = () => selectDirection(1);
document.getElementById("btnDirU").onclick    = () => selectDirection("u");
document.getElementById("btnSubjCat").onclick   = () => selectSubject(1);
document.getElementById("btnSubjHuman").onclick = () => selectSubject(2);
document.getElementById("btnSubjOther").onclick = () => selectSubject(3);
document.getElementById("btnSubjEmpty").onclick = () => selectSubject(0);
document.getElementById("btnSubjU").onclick     = () => selectSubject("u");
document.getElementById("btnPrev").onclick    = () => advance(-1);
document.getElementById("btnSkip").onclick    = () => advance(+1);
document.getElementById("btnSave").onclick    = () => save();

(async () => {
  const init = await fetchJSON("/api/init");
  document.getElementById("user").textContent = init.user;
  document.getElementById("filter").textContent = init.filter + " (" + init.queue.length + " bursts)";
  state.queue = init.queue;
  if (state.queue.length === 0) {
    document.body.innerHTML += "<p>Queue is empty for filter: " + init.filter + "</p>";
    return;
  }
  await loadIdx(0);
})();
</script>
</body></html>"""


def make_handler(filter_mode: str):
    class Handler(server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return  # quiet

        def _json(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urllib.parse.urlparse(self.path)
            if url.path == "/" or url.path == "/index.html":
                body = HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if url.path == "/api/init":
                queue = []
                for x in BURSTS:
                    rep_id = f"captures/sd/{x['burst_id']}/{x['rep_name']}"
                    queue.append({**x, "existing": existing_labels(rep_id)})
                return self._json(200, {
                    "user": USER,
                    "filter": filter_mode,
                    "queue": queue,
                })
            if url.path.startswith("/image/"):
                rel = url.path[len("/image/"):]
                # Security: only allow paths under captures/sd/
                target = (REPO / rel).resolve()
                try:
                    target.relative_to(SD.resolve())
                except ValueError:
                    self.send_error(403)
                    return
                if not target.exists() or not target.is_file():
                    self.send_error(404)
                    return
                data = target.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "max-age=3600")
                self.end_headers()
                self.wfile.write(data)
                return
            self.send_error(404)

        def do_POST(self):
            if self.path != "/api/label":
                self.send_error(404)
                return
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length))
            burst_id = body["burst_id"]
            prey = body.get("prey")
            direction = body.get("direction")
            subject = body.get("subject")
            notes = body.get("notes")
            burst_dir = SD / burst_id
            meta = burst_meta(burst_dir) or {}
            images = meta.get("images", [])
            now = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
            added = 0
            with LABELS.open("a") as f:
                for img in images:
                    name = img.get("f")
                    if not name:
                        continue
                    image_id = f"captures/sd/{burst_id}/{name}"
                    if prey is not None:
                        rec = {
                            "image_id": image_id,
                            "source": f"human:{USER}:burst_prey",
                            "label": (1 if prey == 1 else 0) if prey != "u" else 0,
                            "confidence": 1.0 if prey != "u" else 0.0,
                            "ts": now,
                            "notes": (notes or "") + (" [unclear]" if prey == "u" else ""),
                        }
                        f.write(json.dumps(rec, sort_keys=True) + "\n")
                        added += 1
                    if direction is not None:
                        rec = {
                            "image_id": image_id,
                            "source": f"human:{USER}:burst_direction",
                            "label": (1 if direction == 1 else 0) if direction != "u" else 0,
                            "confidence": 1.0 if direction != "u" else 0.0,
                            "ts": now,
                            "notes": (notes or "") + (" [unclear]" if direction == "u" else ""),
                        }
                        f.write(json.dumps(rec, sort_keys=True) + "\n")
                        added += 1
                    if subject is not None:
                        rec = {
                            "image_id": image_id,
                            "source": f"human:{USER}:burst_subject",
                            "label": int(subject) if subject != "u" else 0,
                            "confidence": 1.0 if subject != "u" else 0.0,
                            "ts": now,
                            "notes": (notes or "") + (" [unclear]" if subject == "u" else ""),
                        }
                        f.write(json.dumps(rec, sort_keys=True) + "\n")
                        added += 1
            rep_name = next(
                (img.get("f") for img in images if img.get("f")), images[0].get("f")
                if images else "")
            rep_id = f"captures/sd/{burst_id}/{rep_name}"
            # Refresh in-memory index so subsequent loads see fresh labels.
            rebuild_label_index()
            return self._json(200, {"added": added, "existing": existing_labels(rep_id)})

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument(
        "--filter",
        default="prey-positive",
        choices=["prey-positive", "disagreement", "unlabelled", "all",
                 "human", "other"],
    )
    ap.add_argument("--since", help="YYYYMMDD, inclusive lower bound on burst date")
    ap.add_argument("--until", help="YYYYMMDD, inclusive upper bound on burst date")
    args = ap.parse_args()

    print(f"Building queue (filter={args.filter})...", flush=True)
    rebuild_label_index()
    print(f"  loaded {len(LABEL_INDEX)} labelled image entries", flush=True)
    global BURSTS
    BURSTS = build_queue(args.filter, since=args.since, until=args.until)
    print(f"  -> {len(BURSTS)} bursts in queue", flush=True)
    if not BURSTS:
        print("Nothing to label. Try --filter all", flush=True)
        return

    print(f"\nServing http://127.0.0.1:{args.port}/   (Ctrl-C to stop)\n",
          flush=True)
    Handler = make_handler(args.filter)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", args.port), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nbye.")


if __name__ == "__main__":
    main()
