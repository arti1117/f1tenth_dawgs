#!/bin/bash
# Wrapper script to generate trajectories with proper ROS2 environment

set -e

# Source ROS2 workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "Error: ROS2 workspace not built. Please run 'colcon build' first."
    exit 1
fi

# Path to generator script
GENERATOR="$WS_DIR/src/race_stack/planner/global_planner/global_planner/offline_trajectory_generator.py"

if [ ! -f "$GENERATOR" ]; then
    echo "Error: Generator script not found at $GENERATOR"
    exit 1
fi

# Show usage if no arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Generate global trajectory JSON from PNG/YAML map files (no ROS topics needed)"
    echo ""
    echo "Examples:"
    echo "  # Generate for specific map"
    echo "  $0 --map_name hangar_1905_v0 \\"
    echo "     --map_dir $WS_DIR/src/race_stack/stack_master/maps/hangar_1905_v0"
    echo ""
    echo "  # With custom parameters"
    echo "  $0 --map_name my_map \\"
    echo "     --map_dir /path/to/map \\"
    echo "     --safety_width 0.30 \\"
    echo "     --safety_width_sp 0.25 \\"
    echo "     --reverse"
    echo ""
    echo "  # Batch process all maps"
    echo "  $WS_DIR/src/race_stack/planner/global_planner/scripts/batch_generate_trajectories.sh \\"
    echo "     $WS_DIR/src/race_stack/stack_master/maps"
    echo ""
    python3 "$GENERATOR" --help
    exit 0
fi

# Run generator with all arguments
python3 "$GENERATOR" "$@"
