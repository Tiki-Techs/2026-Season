# Autoclimb System

Autonomous Level 1 climb sequence triggered from the driver controller.

## Prerequisite

**`calibrateClimb()` must have run this power cycle** before triggering autoclimb.
The climber encoder is not absolute; without calibration the position control targets
(rung height, lift) will be wrong and the robot will not lift.

Run calibration from the autonomous chooser or bind it to a button before the match.

## Button

| Button | Action |
|--------|--------|
| Driver **X** | Hold to run autoclimb. Release at any time to abort. |
| Driver **Back (⊟)** | Reset field heading (moved from X to free that button) |

## State Sequence

```
IDLE → PATHFINDING → FINAL_APPROACH → ALIGNED_HOLD → ENGAGING → LIFTING → CLIMBED
```

| State | What happens |
|-------|-------------|
| PATHFINDING | PathPlanner navigates to the standoff pose (~1.5 m from target). Timeout: 4 s. |
| FINAL_APPROACH | PID drives to capture pose; vision switches to single-tag mode for precision. Proceeds on alignment (X ±2 cm, Y ±5 cm, heading ±1°) debounced for 0.25 s, or on 3 s timeout. |
| ALIGNED_HOLD | Holds position for 1 s to confirm alignment before engaging. |
| ENGAGING | Drives backward (robot frame −X) at 0.15 m/s until: distance ≥ 10 cm, OR drivetrain stall current ≥ 30 A for 0.1 s, OR 2 s timeout. Timeout → ABORTED; distance/current → LIFTING. |
| LIFTING | Raises climber to rung height, then lifts robot. Timeout: 5 s. |
| CLIMBED | Terminal. Wheels lock. |

## Abort

Releasing the **X** button at any point:
- Triggers `emergencyRetract()` — duty-cycle retracts climber to upper limit
- Resumes `FUSED_MEGATAG2` vision mode
- Unlocks the drivetrain (default drive command resumes)

## Target Selection

The default target is the inboard (scoring-side) rung on Tag 15 (Red) or Tag 31 (Blue).
Alliance is read from the Driver Station at command initialization.
If alliance is unknown, Blue Tag 31 Scoring is used.

## SmartDashboard Keys

All keys are under `/Autoclimb/`:

| Key | Description |
|-----|-------------|
| `Autoclimb/State` | Current state name |
| `Autoclimb/Target` | Selected engagement target |
| `Autoclimb/TargetX` / `TargetY` | Capture pose coordinates (m) |
| `Autoclimb/PoseErrorX` / `PoseErrorY` | Translation error to capture pose (m) |
| `Autoclimb/HeadingErrorDeg` | Heading error to target (degrees) |
| `Autoclimb/EngageDistM` | Distance traveled during ENGAGING (m) |
| `Autoclimb/AvgStatorA` | Average drivetrain drive stator current (A) |
| `Autoclimb/ExitReason` | What ended ENGAGING: `distance`, `current`, or `timeout` |

## Values That Need Physical Tuning Before Competition

These constants in [AutoclimbConstants.java](../src/main/java/frc/robot/commands/autoclimb/AutoclimbConstants.java)
are marked `PLACEHOLDER` and must be set from CAD measurements and robot testing:

| Constant | Default | Notes |
|----------|---------|-------|
| `CLIMBER_OFFSET_FROM_ROBOT_CENTER` | (−12.0″, 0) | Measure from CAD: robot center → climber O center |
| `CLIMBER_RUNG_HEIGHT_ROTATIONS` | −5.0 | Motor rotations when O is at rung height |
| `CLIMBER_STOW_ROTATIONS` | −0.3 | Motor rotations for travel stow |
| `CLIMBER_KP` | 2.0 | Slot 0 kP for PositionVoltage control — tune with Phoenix Tuner X |
