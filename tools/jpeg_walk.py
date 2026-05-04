"""Walk a JPEG and print marker structure."""
import struct
import sys

data = open(sys.argv[1] if len(sys.argv) > 1 else
            "captures/pipetest_out/f05_A.jpg", "rb").read()
print(f"File size: {len(data)}")

i = 0
while i < len(data) - 1:
    if data[i] != 0xFF:
        i += 1
        continue
    m = data[i + 1]
    if m == 0xD8:
        print(f"  {i:6} SOI")
        i += 2
    elif m == 0xD9:
        print(f"  {i:6} EOI")
        break
    elif m == 0xDA:
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        ncomp = data[i + 4]
        comps = []
        for c in range(ncomp):
            cid = data[i + 5 + c * 2]
            tab = data[i + 5 + c * 2 + 1]
            comps.append(f"id={cid} dc/ac=0x{tab:02x}")
        print(f"  {i:6} SOS len={seg_len} ncomp={ncomp} {comps}")
        i += 2 + seg_len
    elif m == 0xC0:
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        seg = data[i:i + 2 + seg_len]
        prec = seg[4]
        h = struct.unpack(">H", seg[5:7])[0]
        w = struct.unpack(">H", seg[7:9])[0]
        nc = seg[9]
        comps = []
        for c in range(nc):
            cid = seg[10 + c * 3]
            samp = seg[11 + c * 3]
            qid = seg[12 + c * 3]
            comps.append(f"id={cid} samp=0x{samp:02x}(h={samp>>4},v={samp&15}) qt={qid}")
        print(f"  {i:6} SOF0 prec={prec} {w}x{h} ncomp={nc} {comps}")
        i += 2 + seg_len
    elif m == 0xC4:
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        tc_th = data[i + 4]
        print(f"  {i:6} DHT len={seg_len} class/dest=0x{tc_th:02x} "
              f"(class={tc_th>>4} dest={tc_th&15})")
        i += 2 + seg_len
    elif m == 0xDB:
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        pq_tq = data[i + 4]
        print(f"  {i:6} DQT len={seg_len} prec/dest=0x{pq_tq:02x}")
        i += 2 + seg_len
    elif m in (0xE0, 0xE1, 0xFE):
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        print(f"  {i:6} APP/COM 0x{m:02x} len={seg_len}")
        i += 2 + seg_len
    else:
        print(f"  {i:6} marker 0x{m:02x}")
        i += 2
