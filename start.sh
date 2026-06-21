#!/bin/bash
# ---------------------------------------------------------------------------
# launch.sh — Start camera, TagSLAM, and PID control nodes
# Usage: ./launch.sh [options]
#
# Options:
#   --no-camera     Skip usb_cam (if images come from another source)
#   --sim-time      Enable use_sim_time (for rosbag playback)
#   --dry-run       Print commands without executing
# ---------------------------------------------------------------------------
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
INSTALL_DIR="${SCRIPT_DIR}/install"

# ---------- Configurable parameters ----------
CAM_DEVICE="/dev/video0"
CAM_WIDTH=1920
CAM_HEIGHT=1200
CAM_FPS=30
USE_SIM_TIME="False"
SKIP_CAMERA=0

# ---------- Parse arguments ----------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-camera)  SKIP_CAMERA=1; shift ;;
        --sim-time)   USE_SIM_TIME="True"; shift ;;
        --dry-run)    DRY_RUN=1; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ---------- Check prerequisites ----------
if [ ! -d "$INSTALL_DIR" ]; then
    echo "[ERROR] Install directory not found. Run: colcon build --symlink-install"
    exit 1
fi

if [ ! -f "$CONFIG_DIR/cameras.yaml" ]; then
    echo "[ERROR] Config not found: $CONFIG_DIR/cameras.yaml"
    exit 1
fi

# ---------- Source ROS2 environment ----------
echo "=== Sourcing ROS2 environment ==="
if [ -z "$ROS_DISTRO" ]; then
    # Try common install locations
    for d in /opt/ros/humble /opt/ros/jazzy /opt/ros/rolling; do
        if [ -f "$d/setup.bash" ]; then
            source "$d/setup.bash"
            break
        fi
    done
    # Fallback: conda env
    if [ -z "$ROS_DISTRO" ] && [ -d "$HOME/miniforge3/envs/ros2_humble" ]; then
        source "$HOME/miniforge3/envs/ros2_humble/setup.bash"
    fi
fi
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 not found. Source your ROS2 setup.bash first."
    exit 1
fi
echo "  ROS_DISTRO=$ROS_DISTRO"

source "${INSTALL_DIR}/setup.bash" 2>/dev/null || source "${INSTALL_DIR}/setup.zsh"

# Ensure conda ROS2 library path is available (needed for robostack installs)
if [ -n "$CONDA_PREFIX" ] && [ -d "$CONDA_PREFIX/lib" ]; then
    export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$CONDA_PREFIX/opt/rviz_ogre_vendor/lib:${LD_LIBRARY_PATH}"
fi

# ---------- Helper: run a node in background ----------
launch_bg() {
    local name="$1"; shift
    echo "  [${name}] $*"
    if [ "${DRY_RUN:-0}" = "1" ]; then
        return
    fi
    "$@" &
    local pid=$!
    sleep 0.5
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[ERROR] ${name} failed to start"
        return 1
    fi
}

# ---------- Cleanup on exit ----------
cleanup() {
    echo ""
    echo "=== Shutting down ==="
    jobs -p | xargs -r kill 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

# =============================  1. Camera  =============================
if [ "$SKIP_CAMERA" = "0" ]; then
    echo "=== 1. Starting camera (usb_cam) ==="
    if ! ros2 pkg list 2>/dev/null | grep -q usb_cam; then
        echo "[WARN] usb_cam not installed. Install it first:"
        echo "        sudo apt install ros-\${ROS_DISTRO}-usb-cam"
        echo "        (or build from source in src/)"
        echo "  Camera will be SKIPPED."
    elif [ ! -e "$CAM_DEVICE" ]; then
        echo "[WARN] $CAM_DEVICE not found. Camera SKIPPED."
    else
        launch_bg "camera" \
            ros2 run usb_cam usb_cam_node_exe --ros-args \
                -p video_device:="${CAM_DEVICE}" \
                -p image_width:=${CAM_WIDTH} \
                -p image_height:=${CAM_HEIGHT} \
                -p framerate:=${CAM_FPS} \
                -r __ns:=/camera
    fi
else
    echo "=== 1. Camera SKIPPED (--no-camera) ==="
fi

# ===========================  2. Sync & Detect  ========================
echo "=== 2. Starting sync_and_detect ==="
launch_bg "sync_detect" \
    ros2 run tagslam sync_and_detect_node --ros-args \
        -p "cameras:=${CONFIG_DIR}/cameras.yaml" \
        -p "tagslam_config:=${CONFIG_DIR}/tagslam.yaml" \
        -p "use_sim_time:=${USE_SIM_TIME}"

# ===========================  3. TagSLAM  ===============================
echo "=== 3. Starting tagslam ==="
launch_bg "tagslam" \
    ros2 run tagslam tagslam_node --ros-args \
        -p "cameras:=${CONFIG_DIR}/cameras.yaml" \
        -p "camera_poses:=${CONFIG_DIR}/camera_poses.yaml" \
        -p "tagslam_config:=${CONFIG_DIR}/tagslam.yaml" \
        -p "use_sim_time:=${USE_SIM_TIME}" \
        -p "use_approximate_sync:=True"

# =========================  4. PID Control  =============================
echo "=== 4. Starting PID control ==="
launch_bg "pid" \
    ros2 run pid_control pid_node.py --ros-args \
        -p "config_file:=${CONFIG_DIR}/pid_control.yaml" \
        -p "tagslam_config:=${CONFIG_DIR}/tagslam.yaml"

echo ""
echo "=============================================="
echo "  All nodes started. Press Ctrl+C to stop."
echo "=============================================="

# Wait for any background job to exit (or Ctrl+C)
wait
