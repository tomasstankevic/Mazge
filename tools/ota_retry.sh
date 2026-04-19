#!/bin/bash
set +H
cd /Users/ruta/Tomas/repos/Mazge
for i in $(seq 1 10); do
    echo "=== Attempt $i at $(date +%H:%M:%S) ==="
    python3 tools/espota_fixed.py -i 192.168.0.41 -p 3232 -f .pio/build/ota/firmware.bin -t 120 -d 2>&1
    RET=$?
    if [[ $RET -eq 0 ]]; then
        echo "SUCCESS!"
        break
    fi
    echo "Failed exit=$RET, waiting 10s..."
    sleep 10
done
# Verify
sleep 15
echo "=== Checking firmware ==="
curl -s --max-time 5 http://192.168.0.41/ 2>/dev/null | grep -o 'value="[0-9]*"' | head -3
curl -s --max-time 5 http://192.168.0.41/stats 2>/dev/null
echo ""
