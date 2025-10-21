#!/usr/bin/env python3
from robstride import Motor
import os
from dotenv import load_dotenv

load_dotenv()
PORT = os.getenv('PORT', '/dev/ch340_can')
BITRATE = int(os.getenv('BITRATE', 1000000))
NODE_ID = int(os.getenv('NODE_ID', 107))

print(f"Connecting to motor {NODE_ID} on {PORT} at {BITRATE} bps...")
motor = Motor(port=PORT, motor_id=NODE_ID, bitrate=BITRATE)
motor.connect()

pos = motor.get_position()
vel = motor.get_velocity()
temp = motor.get_temperature()

print(f"Pos: {pos:.3f} rad")
print(f"Vel: {vel:.3f} rad/s")
print(f"Temp: {temp}°C")

motor.disconnect()
print("Done.")

