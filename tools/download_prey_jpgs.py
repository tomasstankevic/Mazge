"""Download all JPGs for the prey-positive bursts using /sdlist?dir=<folder>.

Robust against both old (f00.jpg) and new (f00_0100ms.jpg) filename schemes.
"""
import json
import re
import sys
import urllib.request
import urllib.parse
from pathlib import Path

HOST = "192.168.0.41"
LOCAL = Path("captures/prey_review")

PREY = [
    "20260501_015148_gen1", "20260504_021353_gen10", "20260504_173921_gen1",
    "20260505_005540_gen1", "20260505_025335_gen1", "20260505_212843_gen1",
    "20260505_220846_gen8", "20260505_221411_gen11", "20260505_230846_gen22",
    "20260505_231157_gen23", "20260505_232424_gen25", "20260505_232532_gen27",
    "20260505_234028_gen30", "20260505_235026_gen33", "20260506_001046_gen38",
    "20260506_001106_gen39", "20260506_013555_gen1", "20260506_045151_gen2",
    "20260506_183727_gen2", "20260507_032001_gen3", "20260507_050927_gen5",
    "20260507_080536_gen3", "20260507_212120_gen9", "20260507_232821_gen12",
    "20260508_035852_gen1",
]


def list_dir(folder):
    url = f"http://{HOST}/sdlist?dir={urllib.parse.quote(folder)}"
    with urllib.request.urlopen(url, timeout=30) as r:
        text = r.read().decode("utf-8", errors="replace")
    try:
        d = json.loads(text)
        files = d.get("files", [])
    except json.JSONDecodeError:
        files = re.findall(r'"([^"]+)"', text)
    out = []
    for f in files:
        if f.startswith(folder + "/"):
            out.append(f[len(folder) + 1:])
        else:
            out.append(f)
    return out


def download(folder, fn, dest):
    url = f"http://{HOST}/sdget?f={urllib.parse.quote(folder + '/' + fn)}"
    try:
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        dest.write_bytes(data)
        return len(data)
    except Exception as e:
        return f"ERR {e}"


total_new = 0
for p in PREY:
    out = LOCAL / p
    out.mkdir(parents=True, exist_ok=True)
    have = {f.name for f in out.iterdir() if f.is_file()}
    try:
        files = list_dir(p)
    except Exception as e:
        print(f"{p}: list error {e}")
        continue
    new = sorted(f for f in files if f not in have)
    print(f"{p}  remote={len(files)} have={len(have)} new={len(new)}")
    for fn in new:
        size = download(p, fn, out / fn)
        if isinstance(size, int):
            total_new += 1
            print(f"  +{fn}: {size}")
        else:
            print(f"  !{fn}: {size}")

print(f"\nTotal downloaded: {total_new}")
