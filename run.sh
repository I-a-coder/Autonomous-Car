#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

echo "=== Cleaning up previous Gazebo session ==="

# Kill all related processes
pkill -f "gz sim|ros_gz_bridge|gzserver|gzclient" 2>/dev/null || true
pkill -f "ros2 launch.*city.launch" 2>/dev/null || true
sleep 1

# Purge all Gazebo Sim cache/state so the car spawns fresh at (0,0)
rm -rf ~/.gz/sim/

echo "=== Starting launch ==="

ros2 launch robotics_project city.launch.py "$@"
