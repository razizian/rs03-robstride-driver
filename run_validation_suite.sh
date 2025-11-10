#!/bin/bash
# RS03 Motor Validation Suite
# Runs through all validation steps in sequence

set -e  # Exit on error

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}RS03 Motor System Validation Suite${NC}"
echo "=================================="
echo ""

# Check if CAN interface is up
if ! ip link show can0 &> /dev/null; then
    echo -e "${RED}ERROR: CAN interface 'can0' not found${NC}"
    echo "Please run: ./scripts/can_up.sh"
    exit 1
fi

# Check if running with appropriate permissions
if ! groups | grep -q "dialout\|can"; then
    echo -e "${YELLOW}WARNING: User may not have CAN permissions${NC}"
fi

# Function to run a test
run_test() {
    local test_name=$1
    local test_cmd=$2
    
    echo ""
    echo -e "${YELLOW}Running: $test_name${NC}"
    echo "----------------------------------------"
    
    if $test_cmd; then
        echo -e "${GREEN}✓ $test_name completed successfully${NC}"
        return 0
    else
        echo -e "${RED}✗ $test_name failed${NC}"
        return 1
    fi
}

# Ask user for motor ID if not set
if [ -z "$TEST_MOTOR_ID" ]; then
    echo -n "Enter motor ID to test (default: 101): "
    read motor_id
    export TEST_MOTOR_ID=${motor_id:-101}
fi

echo ""
echo "Testing with Motor ID: $TEST_MOTOR_ID"
echo ""

# Step 1: Hardware Validation
echo -e "${YELLOW}STEP 1: Hardware Validation${NC}"
echo "This will test basic motor functionality with reduced safety limits"
echo "Press Enter to continue or Ctrl+C to cancel..."
read

if run_test "Hardware Validation" "uv run python examples/hardware_validation.py"; then
    echo ""
    echo "Hardware validation passed. Motor is responding correctly."
else
    echo ""
    echo -e "${RED}Hardware validation failed. Please check:${NC}"
    echo "  1. Motor power and connections"
    echo "  2. CAN bus termination"
    echo "  3. Motor ID configuration"
    exit 1
fi

# Step 2: Multi-Motor Incremental Test
echo ""
echo -e "${YELLOW}STEP 2: Multi-Motor Validation (Optional)${NC}"
echo "This will test multiple motors incrementally (1→2→3→4→5)"
echo -n "Run multi-motor test? (y/N): "
read run_multi

if [[ $run_multi =~ ^[Yy]$ ]]; then
    run_test "Multi-Motor Incremental" "uv run python examples/multi_motor_incremental.py"
fi

# Step 3: Latency Measurement
echo ""
echo -e "${YELLOW}STEP 3: System Latency Profiling${NC}"
echo "This will measure command-to-actuation latency"
echo "Press Enter to continue..."
read

if run_test "Latency Measurement" "uv run python examples/latency_measurement.py"; then
    echo ""
    echo "Latency report saved to: latency_report.json"
    
    # Display summary if jq is available
    if command -v jq &> /dev/null; then
        echo ""
        echo "Quick Summary:"
        jq '.results.single_motor | to_entries | .[] | "\(.key): avg=\(.value."MIT Position".avg_ms // "N/A")ms"' latency_report.json 2>/dev/null || true
    fi
fi

# Step 4: Trajectory Following Demo
echo ""
echo -e "${YELLOW}STEP 4: Trajectory Following Demos${NC}"
echo "This will demonstrate advanced motion control:"
echo "  - Sine wave trajectories"
echo "  - Trapezoidal velocity profiles"  
echo "  - Multi-motor coordination"
echo -n "Run trajectory demos? (y/N): "
read run_traj

if [[ $run_traj =~ ^[Yy]$ ]]; then
    run_test "Trajectory Following" "uv run python examples/trajectory_following.py"
fi

# Summary
echo ""
echo "=================================="
echo -e "${GREEN}Validation Suite Complete!${NC}"
echo ""
echo "Next steps:"
echo "  1. Review latency_report.json for performance metrics"
echo "  2. Adjust safety limits in examples if needed"
echo "  3. Implement your application using the examples as reference"
echo "  4. See docs/cpp_migration_plan.md for C++ SDK roadmap"
echo ""
echo "For ROS2 integration:"
echo "  - cd rs03_driver/"
echo "  - colcon build"
echo "  - ros2 launch rs03_driver rs03.launch.py"
echo ""
