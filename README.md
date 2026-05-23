# Capstan Drive Demo

A proof-of-concept capstan drive actuator using a BLDC motor and [SimpleFOC](https://simplefoc.com/). Built to explore low-backlash, quiet actuation for robotic joints using a rope transmission.

> **Demo video:** [link]

---

## Design

The capstan uses a **D:d ratio of [X:1]** with **[N] turns** of 3mm PE fishing rope.

**Results:**
- Very quiet operation compared to geared alternatives
- Near-zero backlash
- The PE rope stretched noticeably under load. HMPE (Dyneema) or Vectran braid is recommended for anything beyond a demo
- The closed-loop sketch works but doesn't get the full performance out of the SimpleFOC Mini. This is a code implementation issue, not a PID tuning issue

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Teensy 4.0 |
| Motor driver | SimpleFOC Mini |
| Motor | GM4108 (11 pole pairs) |
| Encoder | SameSky AMT10E3, 1280 PPR / 5120 CPR |
| Rope | 3mm PE fishing rope *(stretched, not recommended)* |

---

## Code Examples

Three sketches are included:

| Sketch | Description |
|---|---|
| `encoder_test/` | Verify encoder wiring and CPR before touching the motor |
| `openloop_test/` | Spin the motor open-loop to confirm driver wiring and phase order |
| `closed_loop/` | Closed-loop angle control with homing and gear ratio measurement |

For a clean reference implementation of position control with SimpleFOC, see:
**[SimpleFOC Position Control Example](https://docs.simplefoc.com/position_control_example)**

A serial monitor GUI for the closed-loop sketch is included in the `dashboard/` folder. See the README there for setup.

---

## Closed Loop: How It Works

The sketch runs `MotionControlType::angle` (cascaded position to velocity to voltage) with a few additions on top.

**Homing** measures the gear ratio at runtime by driving the motor between the two physical end-stops and recording how far the motor shaft turned per 180 degrees of output shaft travel. Two modes are supported:

- `H` auto: motor drives itself to each stop
- `M` + `S` + `S` manual: move the output shaft by hand and confirm each position

**Gear ratio measurement:**
```
gearRatio = motorTravelRad / pi
```

Once homed, all `T` commands are in **output shaft degrees** (0 to 180), automatically converted to motor shaft radians.

**Wiring (Teensy 4.0 + SimpleFOC Mini):**

| Signal | Pin |
|---|---|
| Encoder A | D2 |
| Encoder B | D3 |
| PWM Phase U | D10 |
| PWM Phase V | D8 |
| PWM Phase W | D7 |
| Driver enable | D6 |

> The encoder index pin is not used. It caused `initFOC` index search failures on this hardware. After the first successful run, hardcode the sensor offset and direction in `motor.initFOC()` to skip the alignment wiggle.

---

## Serial Commands

Open Serial Monitor at **115200 baud**.

| Command | Action |
|---|---|
| `H` | Auto-home: drive to each end-stop, measure gear ratio |
| `M` | Manual home: disable motor, move shaft to stop 1 by hand |
| `S` | Confirm each manual homing step (send twice) |
| `T<deg>` | Move to output shaft angle in degrees, e.g. `T90` |
| `L<V>` | Set voltage limit, e.g. `L5` |
| `P` | Print current position, target, error, and gear ratio |

After homing: `T0` = stop 1, `T90` = centre, `T180` = stop 2.

---

## Lessons Learned

- **Rope material matters.** PE fishing rope is easy to rig but creeps under load. HMPE/Dyneema gives much better stiffness and near-zero elongation.
- **D:d ratio** sets both the torque multiplication and the minimum wraps needed to prevent slip. More wraps add friction along the rope path.
- **Teensy 4.0 + SimpleFOC Mini.** The hardware is capable but the current implementation doesn't fully utilise the driver. Getting the most out of it needs tighter loop timing and better output scaling.
- **Encoder index.** On this setup, enabling the index pin breaks `initFOC`. Skipping it and hardcoding the calibration result after the first run is the practical workaround.

---

## References

- [SimpleFOC Position Control Example](https://docs.simplefoc.com/position_control_example)
- [SimpleFOC angle loop](https://docs.simplefoc.com/angle_loop)
- [SimpleFOC Mini docs](https://docs.simplefoc.com/simplefocmini)
- [SimpleFOC Commander interface](https://docs.simplefoc.com/commander_interface)

---

## License

MIT
