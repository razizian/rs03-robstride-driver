#!/bin/bash
# Test script for ROS2 driver integration
# This script checks if ROS2 can build and run the rs03_driver package

echo "=== ROS2 RS03 Driver Integration Test ==="
echo

# Check if ROS2 is installed
echo "1. Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 found: $(ros2 --version 2>&1 | head -1)"
    
    # Try to detect ROS2 distribution
    if [ -n "$ROS_DISTRO" ]; then
        echo "✓ ROS2 distribution: $ROS_DISTRO"
    else
        echo "⚠ ROS_DISTRO not set. Please source ROS2 setup:"
        echo "  source /opt/ros/<distro>/setup.bash"
        exit 1
    fi
else
    echo "✗ ROS2 not found. Please install ROS2 first."
    echo "  See: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
echo

# Check Python dependencies
echo "2. Checking Python environment..."
if [ -f ".venv/bin/activate" ]; then
    echo "✓ Virtual environment found"
    source .venv/bin/activate
    
    # Check if robstride is installed
    if python -c "import robstride" 2>/dev/null; then
        echo "✓ robstride SDK installed"
    else
        echo "✗ robstride SDK not found in venv"
        echo "  Run: uv sync --no-extras"
        exit 1
    fi
else
    echo "✗ Virtual environment not found"
    echo "  Run: uv sync --no-extras"
    exit 1
fi
echo

# Create test workspace
echo "3. Creating test workspace..."
TEST_WS="$HOME/rs03_test_ws"
if [ -d "$TEST_WS" ]; then
    echo "⚠ Test workspace already exists. Removing..."
    rm -rf "$TEST_WS"
fi

mkdir -p "$TEST_WS/src"
cd "$TEST_WS/src"

# Link the package
ln -s "$(dirname "$(readlink -f "$0")")/rs03_driver" .
echo "✓ Linked rs03_driver package"
echo

# Try to build
echo "4. Building package..."
cd "$TEST_WS"
colcon build --packages-select rs03_driver 2>&1 | tee build.log

if [ $? -eq 0 ]; then
    echo "✓ Build successful!"
else
    echo "✗ Build failed. Check build.log for errors."
    echo
    echo "Common issues:"
    echo "- Missing dependencies (python3-colcon-common-extensions)"
    echo "- ROS2 environment not properly sourced"
    echo "- Python path issues with virtual environment"
    exit 1
fi
echo

# Source the workspace
echo "5. Sourcing workspace..."
source install/setup.bash
echo "✓ Workspace sourced"
echo

# Check if nodes are available
echo "6. Checking available nodes..."
ros2 pkg executables rs03_driver
echo

# Run basic parameter check
echo "7. Testing node startup (dry run - no hardware)..."
echo "Note: This will fail to connect to hardware, which is expected."
echo
timeout 5 ros2 run rs03_driver actuator_node.py 2>&1 | head -20
echo

# Check message types
echo "8. Checking custom messages..."
if ros2 interface show rs03_driver/msg/MotorCommand &>/dev/null; then
    echo "✓ MotorCommand message found"
    ros2 interface show rs03_driver/msg/MotorCommand
else
    echo "✗ MotorCommand message not found"
fi
echo

if ros2 interface show rs03_driver/msg/MotorStatus &>/dev/null; then
    echo "✓ MotorStatus message found"
    ros2 interface show rs03_driver/msg/MotorStatus
else
    echo "✗ MotorStatus message not found"
fi
echo

# Test launch files
echo "9. Checking launch files..."
if [ -f "src/rs03_driver/launch/rs03.launch.py" ]; then
    echo "✓ Single motor launch file found"
fi

if [ -f "src/rs03_driver/launch/rs03_multi.launch.py" ]; then
    echo "✓ Multi motor launch file found"
fi
echo

# Summary
echo "=== Integration Test Summary ==="
echo
echo "To use the ROS2 driver:"
echo "1. Source ROS2: source /opt/ros/\$ROS_DISTRO/setup.bash"
echo "2. Build workspace: cd $TEST_WS && colcon build"
echo "3. Source workspace: source install/setup.bash"
echo "4. Activate Python env: source $(dirname "$(readlink -f "$0")")/.venv/bin/activate"
echo "5. Launch driver: ros2 launch rs03_driver rs03.launch.py"
echo
echo "For multi-motor setup:"
echo "  ros2 launch rs03_driver rs03_multi.launch.py"
echo
echo "✓ ROS2 integration test complete!"
