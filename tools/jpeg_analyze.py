#!/usr/bin/env python3
"""Analyze JPEG marker structure."""
import struct, sys

path = sys.argv[1] if len(sys.argv) > 1 else "captures/burst_20260419_115354_gen1/frame_00.jpg"
data = open(path, "rb").read()
print(f"File size: {len(data)} bytes")

i = 0
while i < len(data) - 1:
    if data[i] != 0xFF:
        i += 1
        continue
    marker = data[i + 1]
    if marker == 0x00:  # stuffed byte
        i += 2
        continue
    if marker == 0xD8:
        print(f"  0x{i:04x}: SOI")
        i += 2
    elif marker == 0xD9:
        print(f"  0x{i:04x}: EOI")
        i += 2
    elif marker == 0xDA:
        length = struct.unpack(">H", data[i+2:i+4])[0]
        ns = data[i + 4]
        print(f"  0x{i:04x}: SOS length={length} components={ns}")
        scan_start = i + 2 + length
        print(f"    Scan data starts at 0x{scan_start:04x}")
        print(f"    First 16 scan bytes: {data[scan_start:scan_start+16].hex()}")
        break
    elif marker == 0xC0:
        length = struct.unpack(">H", data[i+2:i+4])[0]
        prec = data[i + 4]
        h = struct.unpack(">H", data[i+5:i+7])[0]
        w = struct.unpack(">H", data[i+7:i+9])[0]
        nf = data[i + 9]
        print(f"  0x{i:04x}: SOF0 precision={prec} {w}x{h} components={nf}")
        for c in range(nf):
            ci = data[i + 10 + c * 3]
            sf = data[i + 11 + c * 3]
            tq = data[i + 12 + c * 3]
            print(f"    Component {ci}: sampling={sf>>4}x{sf&0xf} qt={tq}")
        i += 2 + length
    elif marker == 0xC4:
        length = struct.unpack(">H", data[i+2:i+4])[0]
        info = data[i + 4]
        tc = info >> 4
        th = info & 0xf
        # Count symbols
        bits = data[i+5:i+21]
        nsymbols = sum(bits)
        print(f"  0x{i:04x}: DHT length={length} class={'AC' if tc else 'DC'} id={th} symbols={nsymbols}")
        i += 2 + length
    elif marker == 0xDB:
        length = struct.unpack(">H", data[i+2:i+4])[0]
        info = data[i + 4]
        prec = info >> 4
        tid = info & 0xf
        print(f"  0x{i:04x}: DQT length={length} precision={prec} id={tid}")
        i += 2 + length
    elif marker == 0xDD:
        ri = struct.unpack(">H", data[i+4:i+6])[0]
        print(f"  0x{i:04x}: DRI restart_interval={ri}")
        i += 6
    elif marker == 0xE0:
        length = struct.unpack(">H", data[i+2:i+4])[0]
        print(f"  0x{i:04x}: APP0 length={length}")
        i += 2 + length
    elif 0xD0 <= marker <= 0xD7:
        print(f"  0x{i:04x}: RST{marker-0xD0}")
        i += 2
    else:
        if i + 3 < len(data):
            length = struct.unpack(">H", data[i+2:i+4])[0]
        else:
            length = 0
        print(f"  0x{i:04x}: FF{marker:02X} length={length}")
        i += 2 + length
