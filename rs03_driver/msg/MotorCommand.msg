# Motor control command message
# Control modes: 0=Operation, 1=Position, 2=Speed, 3=Current

uint8 mode

# Target values (interpretation depends on mode)
float32 position       # rad (mode 1 or 4)
float32 velocity       # rad/s (mode 2 or 4)
float32 torque         # Nm (mode 3 or 4)

# Operation control mode gains (mode 4 only)
float32 kp             # Position gain
float32 kd             # Velocity gain

