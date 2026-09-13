#!/usr/bin/env bash
# ===================================================================
#  Open the wheel-leg robot in the MuJoCo viewer (WSL / Linux launcher).
#
#      ./view_mujoco.sh
#
#  Needs a display.  Under WSL that means WSLg (DISPLAY=:0), which is
#  available on Windows 11 / recent Windows 10.
# ===================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

cat <<'EOF'

 Launching the MuJoCo viewer...

 Viewer controls:
   space        pause / resume
   backspace    reset to the "home" keyframe
   Tab          toggle the UI panels (motor sliders)
   left drag    orbit      right drag  pan      scroll  zoom
   Esc          quit

EOF

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "[WARN] no DISPLAY / WAYLAND_DISPLAY set." >&2
    echo "       Under WSL, WSLg should provide DISPLAY=:0." >&2
    echo "       Run  python3 tools/viewer_check.py  to diagnose." >&2
    echo
fi

exec python3 tools/view.py
