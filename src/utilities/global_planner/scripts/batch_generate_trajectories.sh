#!/bin/bash
# Batch generate trajectories for multiple maps
# Usage: ./batch_generate_trajectories.sh <maps_directory>

set -e

MAPS_DIR="${1:-$(pwd)/../../../../stack_master/maps}"

if [ ! -d "$MAPS_DIR" ]; then
    echo "Error: Maps directory not found: $MAPS_DIR"
    echo "Usage: $0 <maps_directory>"
    exit 1
fi

echo "==========================================="
echo "Batch Trajectory Generation"
echo "==========================================="
echo "Maps directory: $MAPS_DIR"
echo "==========================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
GENERATOR="$SCRIPT_DIR/../global_planner/offline_trajectory_generator.py"

if [ ! -f "$GENERATOR" ]; then
    echo "Error: Generator script not found: $GENERATOR"
    exit 1
fi

# Counter for statistics
TOTAL=0
SUCCESS=0
FAILED=0

# Find all directories with both PNG and YAML files
for MAP_DIR in "$MAPS_DIR"/*/; do
    MAP_NAME=$(basename "$MAP_DIR")

    # Check if PNG and YAML exist
    PNG_FILE="$MAP_DIR/${MAP_NAME}.png"
    YAML_FILE="$MAP_DIR/${MAP_NAME}.yaml"

    if [ -f "$PNG_FILE" ] && [ -f "$YAML_FILE" ]; then
        echo "-------------------------------------------"
        echo "Processing: $MAP_NAME"
        echo "-------------------------------------------"

        TOTAL=$((TOTAL + 1))

        # Run generator
        if python3 "$GENERATOR" \
            --map_name "$MAP_NAME" \
            --map_dir "$MAP_DIR" \
            --no-plots; then
            SUCCESS=$((SUCCESS + 1))
            echo "✓ Success: $MAP_NAME"
        else
            FAILED=$((FAILED + 1))
            echo "✗ Failed: $MAP_NAME"
        fi

        echo ""
    else
        echo "Skipping $MAP_NAME (missing PNG or YAML)"
    fi
done

# Print summary
echo "==========================================="
echo "Summary"
echo "==========================================="
echo "Total maps processed: $TOTAL"
echo "Successful: $SUCCESS"
echo "Failed: $FAILED"
echo "==========================================="

exit 0
