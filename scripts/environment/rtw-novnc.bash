#!/bin/bash
# rtw-novnc
# Starts a virtual X display (Xvfb) on :99, a minimal window manager,
# a VNC server bound to it, and noVNC (browser client) in front of it.
# Runs natively on Ubuntu (no Docker required).
#
# Browser access:   http://localhost:6080/vnc.html
# Direct VNC view:  localhost:5900 (any VNC client)
#
# Usage:
#   rtw-novnc start     # start everything (default)
#   rtw-novnc stop      # stop everything
#   rtw-novnc status    # check what's running
#   rtw-novnc restart

set -e

DISPLAY_NUM="${DISPLAY_NUM:-99}"
export DISPLAY=":${DISPLAY_NUM}"
SCREEN_GEOMETRY="${SCREEN_GEOMETRY:-1920x1080x24}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

PID_DIR="/tmp/novnc-native"
mkdir -p "${PID_DIR}"

check_deps() {
    for cmd in Xvfb fluxbox x11vnc websockify xdpyinfo; do
        command -v "$cmd" >/dev/null 2>&1 || {
            echo "[novnc] Missing dependency: $cmd"
            echo "[novnc] Install with: sudo apt install -y xvfb x11vnc fluxbox novnc websockify x11-apps"
            exit 1
        }
    done
}

start_novnc() {
    check_deps

    if [ -e "/tmp/.X${DISPLAY_NUM}-lock" ]; then
        echo "[novnc] Display :${DISPLAY_NUM} already appears to be in use."
        echo "[novnc] Run '$0 stop' first, or choose a different DISPLAY_NUM."
        exit 1
    fi

    echo "[novnc] Starting Xvfb on ${DISPLAY} (${SCREEN_GEOMETRY})"
    Xvfb "${DISPLAY}" -screen 0 "${SCREEN_GEOMETRY}" -nolisten tcp &
    echo $! > "${PID_DIR}/xvfb.pid"

    for _ in $(seq 1 20); do
        xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1 && break
        sleep 0.25
    done
    if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        echo "[novnc] Xvfb failed to start."
        exit 1
    fi

    echo "[novnc] Starting fluxbox window manager"
    DISPLAY="${DISPLAY}" fluxbox >/dev/null 2>&1 &
    echo $! > "${PID_DIR}/fluxbox.pid"

    echo "[novnc] Starting x11vnc on port ${VNC_PORT}"
    x11vnc -display "${DISPLAY}" -nopw -forever -shared -xkb \
           -rfbport "${VNC_PORT}" -bg -quiet -o "${PID_DIR}/x11vnc.log"
    # x11vnc backgrounds itself with -bg; find its pid via pgrep bound to our display/port
    pgrep -f "x11vnc.*-rfbport ${VNC_PORT}" | head -n1 > "${PID_DIR}/x11vnc.pid"

    echo "[novnc] Starting noVNC (websockify) on port ${NOVNC_PORT}"
    websockify --daemon --web=/usr/share/novnc \
        "${NOVNC_PORT}" "localhost:${VNC_PORT}"
    # This websockify build doesn't support --pidfile, so find the pid ourselves.
    sleep 0.5
    pgrep -f "websockify.*${NOVNC_PORT}.*localhost:${VNC_PORT}" | head -n1 > "${PID_DIR}/websockify.pid"

    echo "[novnc] Ready -> http://localhost:${NOVNC_PORT}/vnc.html"
    echo "[novnc] To use this display in a new terminal: export DISPLAY=${DISPLAY}"
}

stop_novnc() {
    echo "[novnc] Stopping services..."
    for name in websockify x11vnc fluxbox xvfb; do
        pidfile="${PID_DIR}/${name}.pid"
        if [ -f "${pidfile}" ]; then
            pid="$(cat "${pidfile}")"
            if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
                kill "${pid}" 2>/dev/null || true
                echo "[novnc] Stopped ${name} (pid ${pid})"
            fi
            rm -f "${pidfile}"
        fi
    done

    # Fallback cleanup in case pidfiles were missing/stale
    pkill -f "websockify.*${NOVNC_PORT}" 2>/dev/null || true
    pkill -f "x11vnc.*-rfbport ${VNC_PORT}" 2>/dev/null || true
    pkill -f "fluxbox" 2>/dev/null || true
    pkill -f "Xvfb ${DISPLAY}" 2>/dev/null || true

    rm -f "/tmp/.X${DISPLAY_NUM}-lock" 2>/dev/null || true
    rm -f "/tmp/.X11-unix/X${DISPLAY_NUM}" 2>/dev/null || true

    echo "[novnc] Stopped."
}

status_novnc() {
    echo "[novnc] Display: ${DISPLAY}   VNC port: ${VNC_PORT}   noVNC port: ${NOVNC_PORT}"
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        echo "[novnc] Xvfb:        running"
    else
        echo "[novnc] Xvfb:        not running"
    fi
    pgrep -f "fluxbox" >/dev/null 2>&1 && echo "[novnc] fluxbox:     running" || echo "[novnc] fluxbox:     not running"
    pgrep -f "x11vnc.*-rfbport ${VNC_PORT}" >/dev/null 2>&1 && echo "[novnc] x11vnc:      running" || echo "[novnc] x11vnc:      not running"
    pgrep -f "websockify.*${NOVNC_PORT}" >/dev/null 2>&1 && echo "[novnc] websockify:  running" || echo "[novnc] websockify:  not running"
}

case "${1:-start}" in
    start)
        start_novnc
        ;;
    stop)
        stop_novnc
        ;;
    restart)
        stop_novnc
        sleep 1
        start_novnc
        ;;
    status)
        status_novnc
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
