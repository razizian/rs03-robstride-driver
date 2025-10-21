# Connection Diagram (ASCII)

```
[ Ubuntu PC ]
     |
     | USB
     v
[ CH340 USB-CAN ]
     |  CANH  ===== Twisted pair =====>  CANH
     |  CANL  ===== Twisted pair =====>  CANL
     |                                  [ RS03-EN Driver/Motor ]
     |                                   |  +48VDC  (24–60V ok)
     |                                   |  GND  <── Common Ground
     |
(120Ω termination at CH340 side if switch supports it)
(120Ω termination at motor side)
```

Notes:
- Common ground between PSU, adapter, and motor.
- Bitrate: 1,000,000 bps.
- Keep CAN cable short for first tests (≤1 m).



