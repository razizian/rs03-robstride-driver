# Motor status feedback message

std_msgs/Header header

# State
bool enabled
uint8 fault_code       # 0=OK, non-zero=fault

# Telemetry
float32 position       # rad
float32 velocity       # rad/s
float32 torque         # Nm (if available)
float32 temperature    # Celsius
float32 voltage        # V (if available)
float32 current        # A (if available)

