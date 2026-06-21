#!/bin/bash
# ---------------------------------------------------------------------------
# trigger.sh — Trigger one PID regulation cycle
# Usage: ./trigger.sh
# ---------------------------------------------------------------------------
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Source ROS2 (use already-active environment if present)
if [ -z "$ROS_DISTRO" ]; then
    for d in /opt/ros/humble /opt/ros/jazzy /opt/ros/rolling; do
        if [ -f "$d/setup.bash" ]; then
            source "$d/setup.bash"
            break
        fi
    done
    if [ -z "$ROS_DISTRO" ] && [ -d "$HOME/miniforge3/envs/ros2_humble" ]; then
        source "$HOME/miniforge3/envs/ros2_humble/setup.bash"
    fi
fi
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 not found. Source your ROS2 setup.bash first."
    exit 1
fi

source "${SCRIPT_DIR}/install/setup.bash" 2>/dev/null || \
    source "${SCRIPT_DIR}/install/setup.zsh" 2>/dev/null

ros2 service call /pid_control_node/trigger std_srvs/srv/Trigger
