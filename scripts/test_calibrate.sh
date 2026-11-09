#!/usr/bin/env bash
# Runs calibrate.sh against a fake `ros2` so it can be checked without ROS.
# `ros2 action send_goal` exits 0 even when a goal is rejected or aborted,
# so the script has to read the result text; this proves it does.
set -u
here=$(cd "$(dirname "$0")" && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cat > "$tmp/ros2" <<'SHIM'
#!/usr/bin/env bash
case "$1 $2" in
    "action list") printf '/base/calibrate\n/arm/calibrate\n' ;;
    "action send_goal") echo "Goal finished with status: ${CALIB_STATUS:-SUCCEEDED}" ;;
esac
SHIM
chmod +x "$tmp/ros2"
export PATH="$tmp:$PATH"

if CALIB_STATUS=SUCCEEDED "$here/calibrate.sh" >/dev/null; then
    echo "pass: succeeded goals exit 0"
else
    echo "FAIL: succeeded goals exited nonzero"; exit 1
fi
if CALIB_STATUS=ABORTED "$here/calibrate.sh" >/dev/null; then
    echo "FAIL: aborted goal exited 0"; exit 1
else
    echo "pass: aborted goal exits nonzero"
fi
