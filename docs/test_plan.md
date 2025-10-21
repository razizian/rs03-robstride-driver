# Test Plan & Acceptance

Smoke:
- [ ] scripts/can_up.sh brings up can0/slcan0
- [ ] candump shows traffic, ERR stable

Minimal motion:
- [ ] Enable → jog +0.5 rad/s (1 s) → stop → jog -0.5 rad/s (1 s)
- [ ] Telemetry prints pos/vel/temp

Artifacts:
- [ ] Save candump log (10 s)
- [ ] Screenshot terminal output
- [ ] Commit & push

Pass if:
- No bus errors, motion observed, telemetry sane, logs attached.



