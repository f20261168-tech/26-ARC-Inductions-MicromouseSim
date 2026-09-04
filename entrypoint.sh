#!/bin/bash
# entrypoint.sh
# Boots the virtual-display stack (Xvfb -> fluxbox -> x11vnc -> noVNC),
# waits for each layer to actually be ready before starting the next,
# then hands off to the Pygame/ROS 2 simulation loop in the foreground.
set -o pipefail

source /opt/ros/humble/setup.bash

DISPLAY_NUM="${DISPLAY:-:1}"
SCREEN_GEOMETRY="800x800x16"
VNC_PORT=5900
NOVNC_PORT=8080

PIDS=()

cleanup() {
    echo "[entrypoint] Caught shutdown signal, terminating child processes..."
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    wait 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

wait_for_socket() {
    # Polls a unix socket file until it exists or we give up.
    local socket_path="$1"
    local timeout="${2:-10}"
    local waited=0
    while [ ! -S "$socket_path" ] && [ "$waited" -lt "$timeout" ]; do
        sleep 0.5
        waited=$((waited + 1))
    done
}

wait_for_tcp() {
    # Polls a TCP port on localhost until it accepts connections.
    local port="$1"
    local timeout="${2:-15}"
    local waited=0
    while ! bash -c "echo > /dev/tcp/127.0.0.1/${port}" 2>/dev/null; do
        sleep 0.5
        waited=$((waited + 1))
        if [ "$waited" -ge "$((timeout * 2))" ]; then
            echo "[entrypoint] WARNING: timed out waiting for port ${port}"
            return 1
        fi
    done
    return 0
}

echo "[entrypoint] Starting Xvfb on display ${DISPLAY_NUM} (${SCREEN_GEOMETRY})..."
Xvfb "${DISPLAY_NUM}" -screen 0 "${SCREEN_GEOMETRY}" -nolisten tcp &
XVFB_PID=$!
PIDS+=("$XVFB_PID")

# Xvfb creates /tmp/.X11-unix/X<N> once it's actually ready to accept clients.
X_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUM#:}"
wait_for_socket "$X_SOCKET" 15
export DISPLAY="${DISPLAY_NUM}"

if ! kill -0 "$XVFB_PID" 2>/dev/null; then
    echo "[entrypoint] FATAL: Xvfb failed to start. Aborting."
    exit 1
fi
echo "[entrypoint] Xvfb is up (pid ${XVFB_PID})."

echo "[entrypoint] Starting fluxbox window manager..."
fluxbox >/tmp/fluxbox.log 2>&1 &
FLUXBOX_PID=$!
PIDS+=("$FLUXBOX_PID")
sleep 1

echo "[entrypoint] Starting x11vnc on :${VNC_PORT}..."
x11vnc -display "${DISPLAY_NUM}" -nopw -forever -shared -rfbport "${VNC_PORT}" \
    -quiet -noxdamage >/tmp/x11vnc.log 2>&1 &
X11VNC_PID=$!
PIDS+=("$X11VNC_PID")

if ! wait_for_tcp "$VNC_PORT" 15; then
    echo "[entrypoint] FATAL: x11vnc never opened port ${VNC_PORT}. Check /tmp/x11vnc.log."
    exit 1
fi
echo "[entrypoint] x11vnc is up (pid ${X11VNC_PID})."

echo "[entrypoint] Starting noVNC web client on :${NOVNC_PORT}..."
/usr/share/novnc/utils/launch.sh --vnc "localhost:${VNC_PORT}" --listen "${NOVNC_PORT}" \
    >/tmp/novnc.log 2>&1 &
NOVNC_PID=$!
PIDS+=("$NOVNC_PID")

if ! wait_for_tcp "$NOVNC_PORT" 15; then
    echo "[entrypoint] FATAL: noVNC never opened port ${NOVNC_PORT}. Check /tmp/novnc.log."
    exit 1
fi
echo "[entrypoint] noVNC is up. Open http://localhost:${NOVNC_PORT}/vnc.html in a browser."

echo "[entrypoint] Launching simulation engine..."
cd /workspace || { echo "[entrypoint] FATAL: /workspace not found."; exit 1; }

# Run the sim in the foreground so container lifecycle == sim lifecycle.
# If it crashes, the container exits non-zero and docker-compose's
# `restart: unless-stopped` policy will bring it back up.
python3 simulator/sim_engine.py
SIM_EXIT_CODE=$?

echo "[entrypoint] sim_engine.py exited with code ${SIM_EXIT_CODE}. Cleaning up."
cleanup
exit "$SIM_EXIT_CODE"