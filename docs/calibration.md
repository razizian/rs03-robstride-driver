# Calibration & Basic Setup

1) Power & Wiring
- PSU 24–60 VDC (nominal 48 V), common ground with CH340.
- Ensure 120Ω termination at both ends.

2) IDs & Zero
- Set CAN_ID as needed (default may be 1; we use 107 placeholder).
- Set mechanical zero with the SDK/tool when the joint is in neutral.
- Do NOT switch control modes while running; send stop, then switch.

3) Encoder/Mag Calibration (if reassembled)
- Run encoder/magnet calibration per vendor tool or SDK command.
- After calibration, set zero again; save params (non-volatile).

4) Sanity Checks
- Enable → jog small amplitude → stop.
- Verify feedback (position, velocity, temp).



