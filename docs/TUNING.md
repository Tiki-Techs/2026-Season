# Tuning Guide

## Climber (PositionVoltage)

One gain to tune: **`CLIMBER_KP`** in
[AutoclimbConstants.java](../src/main/java/frc/robot/commands/autoclimb/AutoclimbConstants.java).

The climber uses `PositionVoltage` (CTRE Phoenix 6). No motion profile (cruise velocity,
acceleration, jerk) is required — the motor drives directly toward the target position
with output proportional to the position error.

### Tuning Procedure

1. Run `calibrateClimb()` to zero the encoder at the upper hard stop.
2. Open **Phoenix Tuner X** → select the climb TalonFX → **Control** tab → choose `PositionVoltage`.
3. Command a test target, e.g. `CLIMBER_RUNG_HEIGHT_ROTATIONS` (default −5.0 rot).
4. Increase `kP` until the climber reaches target quickly without oscillating or overshooting.
5. If it overshoots: decrease `kP`, or add a small `kD` in Slot 0.
6. Write the final value into `CLIMBER_KP` in `AutoclimbConstants.java`.

Typical starting range: **kP = 1.0 – 4.0** (volts / rotation).

No `kV` (velocity feed-forward) or `kS` (static friction compensation) are configured.
Add them only if the climber stalls noticeably short of target on the first test.

---

## Soft Limits

| Constant | Value | Notes |
|----------|-------|-------|
| `SOFT_LIMIT_FORWARD_ROTATIONS` | 0.5 | Slightly above zero (upper hard stop) to account for encoder drift |
| `SOFT_LIMIT_REVERSE_ROTATIONS` | −9.5 | Slightly past `CLIMBER_LIFT_ROTATIONS` (−9.2) |

Do not change these without re-measuring hard stop positions on the robot.

---

## Current Limits

Stator and supply limits are both fixed at **40 A**. Do not change.
These are hardware-confirmed safe values for the climb motor.
