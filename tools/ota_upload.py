#!/usr/bin/env python3
"""Custom OTA uploader for ESP32 with auto-retry."""
import socket, sys, time, hashlib
import urllib.request
import urllib.error
from datetime import datetime

ESP_IP = "192.168.0.41"
ESP_PORT = 3232
FW_PATH = ".pio/build/ota/firmware.bin"
CHUNK = 8192
FLASH = 0
MAX_RETRIES = 30
VERIFY_RETRIES = 8
VERIFY_DELAY_S = 2

def log(msg):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}", flush=True)

fw = open(FW_PATH, "rb").read()
fw_len = len(fw)
fw_md5 = hashlib.md5(fw).hexdigest()
log(f"Firmware: {fw_len} bytes ({fw_len/1024:.1f} KB), MD5: {fw_md5}")
log(f"Target: {ESP_IP}:{ESP_PORT}  Bin: {FW_PATH}")


def verify_board_online():
    """Check if board came back after upload/reboot."""
    log(f"Verifying board... (up to {VERIFY_RETRIES} tries, {VERIFY_DELAY_S}s apart)")
    for i in range(VERIFY_RETRIES):
        try:
            resp = urllib.request.urlopen(f"http://{ESP_IP}/stats", timeout=5).read()
            log(f"  verify try {i+1}: HTTP OK ({len(resp)} bytes)")
            return True, resp.decode(errors="replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            log(f"  verify try {i+1}: {type(e).__name__}: {e}")
            time.sleep(VERIFY_DELAY_S)
    return False, ""

def attempt_upload(attempt_num):
    log("Opening local TCP listener for ESP callback...")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", 0))
    local_port = srv.getsockname()[1]
    srv.listen(1)
    srv.settimeout(10)
    log(f"  listener on port {local_port}")

    invite = f"{FLASH} {local_port} {fw_len} {fw_md5}\n"
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (ESP_IP, ESP_PORT)

    log(f"Sending UDP invite to {ESP_IP}:{ESP_PORT} (up to 5 tries, 5s each)...")
    got_ok = False
    for k in range(5):
        log(f"  invite try {k+1}/5...")
        udp.sendto(invite.encode(), remote)
        udp.settimeout(5)
        try:
            data = udp.recv(37).decode()
            log(f"  UDP reply: {data!r}")
            if data == "OK":
                got_ok = True
                break
        except socket.timeout:
            log("  UDP timeout (5s)")
    if not got_ok:
        udp.close()
        srv.close()
        return -1, "No UDP response"
    udp.close()

    log("Waiting for ESP to connect back over TCP...")
    try:
        conn, addr = srv.accept()
        log(f"  TCP connect-back from {addr[0]}:{addr[1]}")
    except socket.timeout:
        srv.close()
        return -1, "No TCP connect-back"

    # Keep a deeper send queue so short Wi-Fi hiccups don't immediately stall us.
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.settimeout(30)

    log(f"Streaming {fw_len/1024:.0f}KB in {CHUNK}-byte chunks...")
    offset = 0
    start = time.time()
    last_log_pct = -10
    try:
        while offset < fw_len:
            end = min(offset + CHUNK, fw_len)
            conn.sendall(fw[offset:end])
            offset = end
            pct = offset * 100 // fw_len
            elapsed = time.time() - start
            speed = offset / elapsed / 1024 if elapsed > 0 else 0
            bar = "=" * (pct // 2) + ">" + " " * (50 - pct // 2)
            sys.stdout.write(f"\r  [{bar}] {pct}%  {offset/1024:.0f}/{fw_len/1024:.0f}KB  {speed:.1f}KB/s")
            sys.stdout.flush()
            if pct >= last_log_pct + 10:
                # newline timestamp checkpoint every 10%
                sys.stdout.write("\n")
                log(f"  progress {pct}% @ {speed:.1f}KB/s (elapsed {elapsed:.1f}s)")
                last_log_pct = pct
    except (BrokenPipeError, ConnectionResetError, socket.timeout) as e:
        sys.stdout.write("\n")
        log(f"  send error at {offset/1024:.0f}KB: {type(e).__name__}: {e}")
        conn.close()
        srv.close()
        return offset, str(e)

    elapsed = time.time() - start
    sys.stdout.write("\n")
    log(f"All data sent in {elapsed:.1f}s ({fw_len/elapsed/1024:.1f}KB/s avg)")
    log("Waiting for final ACK from ESP (up to 30s)...")
    conn.settimeout(30)
    try:
        while True:
            data = conn.recv(32)
            if not data:
                log("  ESP closed socket (no explicit ACK)")
                break
            text = data.decode(errors='replace')
            log(f"  ESP says: {text!r}")
            if "OK" in text:
                conn.close()
                srv.close()
                return fw_len, "OK"
            if "ERR" in text:
                conn.close()
                srv.close()
                return fw_len, f"ESP32 error: {text}"
    except (socket.timeout, Exception) as e:
        log(f"  recv error: {type(e).__name__}: {e}")
        conn.close()
        srv.close()
        return fw_len, f"Result wait failed: {e}"

    conn.close()
    srv.close()
    return fw_len, "connection closed"


best = 0
for i in range(1, MAX_RETRIES + 1):
    log(f"=== Attempt {i}/{MAX_RETRIES} (best so far: {best/1024:.0f}KB / {fw_len/1024:.0f}KB) ===")

    transferred, result = attempt_upload(i)

    if transferred > best:
        best = transferred

    if transferred == fw_len and result in {"OK", "connection closed"}:
        log(f"Upload finished ({result}). Verifying board reboot...")
        ok, stats = verify_board_online()
        if ok:
            log("SUCCESS! Board online after OTA.")
            log(f"Board stats: {stats}")
            sys.exit(0)
        log("Board not responding yet; treating this attempt as uncertain.")
    elif transferred == fw_len and result.startswith("Result wait failed"):
        log("Upload payload sent; final ack missing. Verifying board reboot...")
        ok, stats = verify_board_online()
        if ok:
            log("SUCCESS! Board online after OTA.")
            log(f"Board stats: {stats}")
            sys.exit(0)
        log("Board not responding yet; treating this attempt as uncertain.")

    log(f"Attempt {i} failed at {transferred/1024:.0f}KB: {result}")

    if transferred < 0:
        log("  waiting 5s before next attempt...")
        time.sleep(5)
    else:
        log("  waiting 3s before next attempt...")
        time.sleep(3)

log(f"Failed after {MAX_RETRIES} attempts. Best: {best/1024:.0f}KB / {fw_len/1024:.0f}KB")
sys.exit(1)
