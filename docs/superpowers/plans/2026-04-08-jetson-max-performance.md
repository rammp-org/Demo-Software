# Jetson Max Performance Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `scripts/jetson_max_performance.sh` that permanently enables MAXN power mode and max clocks on Jetson Orin NX / AGX Orin, and wire it into `setup.sh`.

**Architecture:** A standalone bash script detects the Jetson platform, applies `nvpmodel -m 0` and `jetson_clocks`, and installs a systemd service so `jetson_clocks` re-runs on every boot. `setup.sh` calls it at the end, forwarding any `--model` argument.

**Tech Stack:** Bash, `nvpmodel`, `jetson_clocks`, systemd.

______________________________________________________________________

### Task 1: Create `scripts/jetson_max_performance.sh`

**Files:**

- Create: `scripts/jetson_max_performance.sh`

- [ ] **Step 1: Create the script file**

```bash
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
```

Save this as `scripts/jetson_max_performance.sh`.

- [ ] **Step 2: Make the script executable**

```bash
chmod +x scripts/jetson_max_performance.sh
```

- [ ] **Step 3: Verify script syntax**

```bash
bash -n scripts/jetson_max_performance.sh
```

Expected: no output, exit code 0.

- [ ] **Step 4: Verify graceful skip on a non-Jetson host**

```bash
bash scripts/jetson_max_performance.sh
```

Expected output (on any non-Jetson machine):

```
Not a Jetson device — skipping max performance setup.
```

Exit code: 0. No `sudo` calls made.

- [ ] **Step 5: Commit**

```bash
git add scripts/jetson_max_performance.sh
git commit -m "feat: add Jetson max performance setup script (issue #83)"
```

______________________________________________________________________

### Task 2: Wire into `setup.sh`

**Files:**

- Modify: `setup.sh` (add section at the end, before the final "Setup complete" echo)

- [ ] **Step 1: Open `setup.sh` and locate the final echo**

The last two lines of `setup.sh` currently are:

```bash
echo "=== Setup complete ==="
```

- [ ] **Step 2: Insert the Jetson performance call before that line**

Replace:

```bash
echo "=== Setup complete ==="
```

With:

```bash
echo "=== Configuring Jetson max performance mode ==="
bash "${REPO_ROOT}/scripts/jetson_max_performance.sh" "$@"

echo "=== Setup complete ==="
```

`"$@"` forwards any `--model <value>` argument passed to `setup.sh` through to the performance script.

- [ ] **Step 3: Verify syntax of the updated `setup.sh`**

```bash
bash -n setup.sh
```

Expected: no output, exit code 0.

- [ ] **Step 4: Smoke-test `setup.sh` on a non-Jetson machine (dry run of the new section only)**

Since running all of `setup.sh` on a dev machine re-runs the full gRPC/rosdep install, just verify the new lines work in isolation:

```bash
REPO_ROOT="$(pwd)" bash -c '
  echo "=== Configuring Jetson max performance mode ==="
  bash "${REPO_ROOT}/scripts/jetson_max_performance.sh"
  echo "=== Setup complete ==="
'
```

Expected output:

```
=== Configuring Jetson max performance mode ===
Not a Jetson device — skipping max performance setup.
=== Setup complete ===
```

- [ ] **Step 5: Commit**

```bash
git add setup.sh
git commit -m "feat: call Jetson max performance script from setup.sh (issue #83)"
```

______________________________________________________________________

### Task 3: Open PR

**Files:** none (GitHub only)

- [ ] **Step 1: Push the branch**

```bash
git push -u origin HEAD
```

- [ ] **Step 2: Open PR to `dev` per CONTRIBUTING.MD**

```bash
gh pr create \
  --base dev \
  --title "feat: enable Jetson max performance mode on boot (issue #83)" \
  --body "$(cat <<'EOF'
## Summary

- Adds `scripts/jetson_max_performance.sh`: detects Jetson platform, applies `nvpmodel -m 0` (MAXN), runs `jetson_clocks`, and installs a systemd service so clocks are re-applied on every boot.
- Supports Jetson Orin NX (default) and AGX Orin; auto-detects model from `/proc/device-tree/model`.
- Gracefully skips on non-Jetson hosts (dev laptops, CI).
- `setup.sh` calls the script at the end, forwarding `--model` if supplied.

Closes #83.

## Test plan

- [ ] On a dev laptop: `bash scripts/jetson_max_performance.sh` prints skip message, exits 0.
- [ ] On Jetson Orin NX: `sudo bash scripts/jetson_max_performance.sh` completes without error; `sudo nvpmodel -q` shows MAXN; `systemctl is-enabled jetson-max-performance` shows `enabled`.
- [ ] After reboot on Jetson: `sudo jetson_clocks --show` confirms clocks are at max.
- [ ] `bash -n setup.sh` and `bash -n scripts/jetson_max_performance.sh` both pass.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
EOF
)"
```
