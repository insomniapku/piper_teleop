#!/bin/bash
# 快速启动脚本 - Bash版本
# 不需要交互输入

echo "======================================================================"
echo "  Pico VR 数据采集系统 - 快速启动"
echo "======================================================================"
echo ""

# Set the ROS workspace. Override with PIPER_WORKSPACE when needed.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="${PIPER_WORKSPACE:-$(cd "$PACKAGE_ROOT/.." && pwd)}"

if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
elif [ -f "$PACKAGE_ROOT/install/setup.bash" ]; then
  source "$PACKAGE_ROOT/install/setup.bash"
fi

# Create the data directory
OUTPUT_DIR="${PIPER_DEMO_DIR:-$HOME/data/piper_demos}"
mkdir -p "$OUTPUT_DIR"

echo "Starting data collection in mock mode..."
echo
echo "Press Ctrl+C to stop."
echo "======================================================================"
echo

# Start the launch file.
ros2 launch piper_teleop data_collection.launch.py xr_mode:=mock record_camera:=false output_dir:="$OUTPUT_DIR" urdf_path:="${PIPER_URDF_PATH:-}"
