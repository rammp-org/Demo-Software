#!/bin/bash
set -e

# Default model used when auto-detection fails
MODEL_ARG="orin-nx"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)
            MODEL_ARG="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

DEVICE_TREE_MODEL="/proc/device-tree/model"

# Platform check — skip gracefully on non-Jetson hosts
if [[ ! -f "$DEVICE_TREE_MODEL" ]] || ! grep -qi "jetson" "$DEVICE_TREE_MODEL"; then
    echo "Not a Jetson device — skipping max performance setup."
    exit 0
fi

# Model detection — used for logging only; nvpmodel mode 0 = MAXN on both targets
MODEL_STRING=$(cat "$DEVICE_TREE_MODEL")
if echo "$MODEL_STRING" | grep -qi "Orin NX"; then
    DETECTED_MODEL="orin-nx"
elif echo "$MODEL_STRING" | grep -qi "AGX Orin"; then
    DETECTED_MODEL="agx-orin"
else
    DETECTED_MODEL="$MODEL_ARG"
    echo "Could not auto-detect Jetson model — using default: $DETECTED_MODEL"
fi

echo "=== Jetson Max Performance Setup (model: $DETECTED_MODEL) ==="

echo "Setting nvpmodel to MAXN (mode 0)..."
sudo nvpmodel -m 0

echo "Locking clocks with jetson_clocks..."
sudo jetson_clocks

echo "Installing jetson-max-performance systemd service..."
SERVICE_FILE="/etc/systemd/system/jetson-max-performance.service"
sudo tee "$SERVICE_FILE" > /dev/null <<'SERVICE'
[Unit]
Description=Jetson Max Performance (jetson_clocks)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable --now jetson-max-performance.service

echo "=== Jetson max performance setup complete ==="
echo "    nvpmodel mode : $(sudo nvpmodel -q | grep 'NV Power Mode' || echo 'unavailable')"
echo "    Service status: $(systemctl is-enabled jetson-max-performance.service)"
