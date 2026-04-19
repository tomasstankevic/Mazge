#!/usr/bin/env python3
"""Custom OTA uploader for ESP32 with auto-retry."""
import socket, sys, time, os, hashlib

ESP_IP = "192.168.0.41"
ESP_PORT = 3232
FW_PATH = ".pio/build/ota/firmware.bin"
CHUNK = 1024
FLASH = 0
MAX_RETRIES = 30

fw = open(FW_PATH, "rb").read()
fw_len = len(fw)
fw_md5 = hashlib.md5(fw).hexdigest()
print(f"Firmware: {fw_len} bytes ({fw_len/1024:.1f} KB), MD5: {fw_md5}")

def attempt_upload(attempt_num):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", 0))
    local_port = srv.getsockname()[1]
    srv.listen(1)
    srv.settimeout(10)

    invite = f"{FLASH} {local_port} {fw_len} {fw_md5}\n"
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (ESP_IP, ESP_PORT)

    for _ in range(5):
        udp.sendto(invite.encode(), remote)
        udp.settimeout(5)
        try:
            data = udp.recv(37).decode()
            if data == "OK":
                break
        except socket.timeout:
            pass
    else:
        udp.close()
        srv.close()
        return -1, "No UDP response"
    udp.close()

    try:
        conn, addr = srv.accept()
    except socket.timeout:
        srv.close()
        return -1, "No TCP connect-back"

    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    offset = 0
    start = time.time()
    try:
        while offset < fw_len:
            end = min(offset + CHUNK, fw_len)
            conn.sendall(fw[offset:end])
            conn.settimeout(120)
            ack = conn.recv(32)
            offset = end
            pct = offset * 100 // fw_len
            elapsed = time.time() - start
            speed = offset / elapsed / 1024 if elapsed > 0 else 0
            bar = "=" * (pct // 2) + ">" + " " * (50 - pct // 2)
            sys.stdout.write(f"\r  [{bar}] {pct}%  {offset/1024:.0f}/{fw_len/1024:.0f}KB  {speed:.1f}KB/s")
            sys.stdout.flush()
    except (BrokenPipeError, ConnectionResetError, socket.timeout) as e:
        conn.close()
        srv.close()
        return offset, str(e)

    # All data sent - wait for final result
    elapsed = time.time() - start
    print(f"\n  All data sent in {elapsed:.1f}s")
    conn.settimeout(120)
    try:
        while True:
            data = conn.recv(32)
            if not data:
                break
            text = data.decode(errors='replace')
            if "OK" in text:
                conn.close()
                srv.close()
                return fw_len, "OK"
            if "ERR" in text:
                conn.close()
                srv.close()
                return fw_len, f"ESP32 error: {text}"
    except (socket.timeout, Exception) as e:
        conn.close()
        srv.close()
        return fw_len, f"Result: {e}"

    conn.close()
    srv.close()
    return fw_len, "connection closed"


best = 0
for i in range(1, MAX_RETRIES + 1):
    print(f"\n{'='*60}")
    print(f"Attempt {i}/{MAX_RETRIES} (best so far: {best/1024:.0f}KB / {fw_len/1024:.0f}KB)")
    print(f"{'='*60}")

    transferred, result = attempt_upload(i)

    if transferred > best:
        best = transferred

    if transferred == fw_len and result == "OK":
        print(f"\n\nSUCCESS! Firmware uploaded.")
        print("Waiting 15s for reboot...")
        time.sleep(15)
        # Verify
        import urllib.request
        try:
            resp = urllib.request.urlopen(f"http://{ESP_IP}/stats", timeout=5).read()
            print(f"Board stats: {resp.decode()}")
        except:
            print("Board not responding yet (may still be booting)")
        sys.exit(0)

    print(f"\n  Failed at {transferred/1024:.0f}KB: {result}")

    if transferred < 0:
        print("  Waiting 5s...")
        time.sleep(5)
    else:
        print("  Waiting 3s...")
        time.sleep(3)

print(f"\n\nFailed after {MAX_RETRIES} attempts. Best: {best/1024:.0f}KB / {fw_len/1024:.0f}KB")
sys.exit(1)
