#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
log_dir="${workspace}/log/novnc"
display_number=99
display=":${display_number}"

mkdir -p "${log_dir}"

child_pids=()
cleanup() {
  for pid in "${child_pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if pgrep -f "^Xvfb ${display}( |$)" >/dev/null; then
  echo "Display ${display} is already in use by Xvfb." >&2
  exit 1
fi

# Remove only the stale files belonging to the dedicated :99 virtual display.
rm -f "/tmp/.X${display_number}-lock" "/tmp/.X11-unix/X${display_number}"

Xvfb "${display}" \
  -screen 0 1440x900x24 \
  -ac \
  +extension GLX \
  +render \
  -noreset \
  >"${log_dir}/xvfb.log" 2>&1 &
child_pids+=("$!")

for _ in $(seq 1 50); do
  if DISPLAY="${display}" xdpyinfo >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done

if ! DISPLAY="${display}" xdpyinfo >/dev/null 2>&1; then
  echo "Xvfb did not become ready; see ${log_dir}/xvfb.log" >&2
  exit 1
fi

DISPLAY="${display}" openbox >"${log_dir}/openbox.log" 2>&1 &
child_pids+=("$!")

env \
  -u WAYLAND_DISPLAY \
  -u WAYLAND_SOCKET \
  -u XDG_SESSION_TYPE \
  DISPLAY="${display}" \
  x11vnc \
  -display "${display}" \
  -forever \
  -shared \
  -nopw \
  -localhost \
  -rfbport 5900 \
  -noxdamage \
  >"${log_dir}/x11vnc.log" 2>&1 &
child_pids+=("$!")

set +u
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

DISPLAY="${display}" \
QT_QPA_PLATFORM=xcb \
QT_X11_NO_MITSHM=1 \
LIBGL_ALWAYS_SOFTWARE=1 \
MESA_GL_VERSION_OVERRIDE=3.3COMPAT \
ros2 launch h1_2_moveit_config rviz_only.launch.py \
  >"${log_dir}/rviz.log" 2>&1 &
child_pids+=("$!")

websockify \
  --web=/usr/share/novnc/ \
  127.0.0.1:6080 \
  127.0.0.1:5900 \
  >"${log_dir}/websockify.log" 2>&1 &
websockify_pid="$!"
child_pids+=("${websockify_pid}")

echo "RViz browser view: http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
wait "${websockify_pid}"
