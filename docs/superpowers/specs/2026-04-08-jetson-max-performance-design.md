# Jetson Max Performance Mode — Design Spec

**Issue:** #83
**Date:** 2026-04-08
**Status:** Approved

## Summary

Add a script to permanently enable max performance mode on the Jetson that hosts the MEBot GUI. The script applies MAXN power mode via `nvpmodel` and locks all clocks via `jetson_clocks`, then installs a systemd service so clocks are re-applied on every boot. It is called from `setup.sh` so it runs as part of standard environment setup.

## Scope

- Targets Jetson Orin NX (primary) and AGX Orin (secondary).
- Gracefully skips on non-Jetson hosts (dev laptops, CI).
- No changes to firmware, GUI, or ROS nodes.

## Files

| File                                | Change                                    |
| ----------------------------------- | ----------------------------------------- |
| `scripts/jetson_max_performance.sh` | New — standalone performance setup script |
| `setup.sh`                          | Add one call to the new script at the end |

## `scripts/jetson_max_performance.sh`

### Platform detection

Read `/proc/device-tree/model`. If the file does not exist or its contents do not contain the string `"Jetson"`, print a skip message and exit 0. This makes the script a no-op on any non-Jetson host with no side effects.

### Model detection

After confirming we are on a Jetson, grep the model string for `"Orin NX"` or `"AGX Orin"`:

- Match → log which model was detected.
- No match → fall back to the `--model` CLI argument (default: `orin-nx`).

Both Orin NX and AGX Orin use `nvpmodel` mode `0` for MAXN, so the nvpmodel invocation is identical regardless of model. The detection is used for logging clarity only.

### CLI argument

```
jetson_max_performance.sh [--model <orin-nx|agx-orin>]
```

- `--model` is optional; used only when auto-detection does not match a known model string.
- Default: `orin-nx`.

### Steps executed

1. `sudo nvpmodel -m 0` — set power envelope to MAXN (all CPU cores active, full TDP). This setting persists across reboots in nvpmodel's own config, so it does not need to be in the systemd service.
1. `sudo jetson_clocks` — lock all CPU/GPU/EMC clocks to their maximums immediately.
1. Write `/etc/systemd/system/jetson-max-performance.service` (see below).
1. `sudo systemctl daemon-reload && sudo systemctl enable --now jetson-max-performance.service`.

### Systemd service

```ini
[Unit]
Description=Jetson Max Performance (jetson_clocks)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

`jetson_clocks` must be re-applied after every boot because the kernel resets clock governors on startup even though `nvpmodel` mode persists.

## `setup.sh` change

Add a new section at the end of `setup.sh`:

```bash
echo "=== Configuring Jetson max performance mode ==="
bash "${REPO_ROOT}/scripts/jetson_max_performance.sh" "$@"
```

Passing `"$@"` forwards any `--model` argument the user gave to `setup.sh` through to the performance script.

## Error handling

- If `nvpmodel` or `jetson_clocks` are not found (i.e., not a Jetson despite `/proc/device-tree/model` existing), the script exits with a non-zero code and a clear error message. `set -e` in the caller (`setup.sh`) will surface this.
- The script requires `sudo` access. If not available, `sudo` will prompt or fail naturally.

## Success criteria

- Running `setup.sh` on an Orin NX or AGX Orin applies MAXN mode and installs the boot service.
- Running `setup.sh` on a dev laptop prints a skip message and completes normally.
- After a reboot on Jetson, `jetson_clocks` has been applied (verifiable via `sudo jetson_clocks --show`).
- `nvpmodel -q` reports mode 0 (MAXN) after setup.
