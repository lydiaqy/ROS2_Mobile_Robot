#!/usr/bin/env bash

# Find the workspace root relative to this script
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the workspace overlay
if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
    source "$WS_ROOT/install/setup.bash"
    echo "ROS 2 workspace loaded: $WS_ROOT"
else
    echo "Workspace not built yet: $WS_ROOT"
    echo "Run colcon build first."
    return 1 2>/dev/null || exit 1
fi
