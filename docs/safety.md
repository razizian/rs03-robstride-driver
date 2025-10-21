# Safety Checklist (v0)

Soft limits for first motion:
- Current limit: 2 A
- Velocity limit: 2 rad/s
- No load attached; secure motor.

Operational rules:
- Send stop before changing modes.
- Beware back-EMF: do not hard-spin by hand on power-off.
- Keep fingers clear; emergency power-off path accessible.

Bring-up steps (quick):
- [ ] PSU set to safe current limit
- [ ] CAN up @ 1 Mbps; termination verified
- [ ] Enable → jog ±0.5 rad/s for 1 s
- [ ] Telemetry confirms nominal temp/velocity



