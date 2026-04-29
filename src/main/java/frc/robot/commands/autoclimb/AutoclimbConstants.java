package frc.robot.commands.autoclimb;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/**
 * All tunables, field geometry, and tolerances for the autoclimb system.
 *
 * FIELD ORIGIN: top-right corner of field as drawn.
 *   +X toward Red Alliance, +Y away from Scoring Table, +Z up.
 * All constants are in METERS internally. Original inch values noted in comments.
 *
 * Tower geometry from drawing GE-26500 (page 28):
 *   Two uprights, 32.250" centerline-to-centerline
 *   Rung horizontal projection: 5.875" from upright centerline each side
 *   Engage 1.0" inboard from rung end for slip-off margin → 4.875" from upright centerline
 *
 * HEADING CONVENTION:
 *   This file uses WPILib heading (CCW positive, 0° = +X field direction).
 *   Red targets: 180° (robot front toward field center / Blue side, back toward Red wall)
 *   Blue targets: 0°  (robot front toward field center / Red side, back toward Blue wall)
 *   NOTE: The field manual uses a different convention; the spec's "Red=0°, Blue=180°"
 *   maps to WPILib 180° and 0° respectively.
 */
public final class AutoclimbConstants {
    private AutoclimbConstants() {}

    // -------------------------------------------------------------------------
    // FIELD GEOMETRY — Tower upright centerline X positions
    // -------------------------------------------------------------------------

    /** Red tower upright centerline X. Original: 609.465 inches */
    public static final double UPRIGHT_X_RED_M = Units.inchesToMeters(609.465);

    /** Blue tower upright centerline X. Original: 42.075 inches */
    public static final double UPRIGHT_X_BLUE_M = Units.inchesToMeters(42.075);

    // -------------------------------------------------------------------------
    // ENGAGEMENT POINTS — Translation2d in field coordinates (meters)
    // Naming: (alliance)_(tag)_(side relative to upright center, from field top)
    // AUDIENCE = +Y side of upright, SCORING = -Y side of upright
    //
    // Original inch values in parentheses.
    // -------------------------------------------------------------------------

    /** Red Tag 15, Audience side. Original: (609.465", 182.720") */
    public static final Translation2d ENGAGE_RED_TAG15_AUDIENCE =
        new Translation2d(Units.inchesToMeters(609.465), Units.inchesToMeters(182.720));

    /** Red Tag 15, Scoring side. Original: (609.465", 172.970") */
    public static final Translation2d ENGAGE_RED_TAG15_SCORING =
        new Translation2d(Units.inchesToMeters(609.465), Units.inchesToMeters(172.970));

    /** Red Tag 16, Audience side. Original: (609.465", 150.470") */
    public static final Translation2d ENGAGE_RED_TAG16_AUDIENCE =
        new Translation2d(Units.inchesToMeters(609.465), Units.inchesToMeters(150.470));

    /** Red Tag 16, Scoring side. Original: (609.465", 140.720") */
    public static final Translation2d ENGAGE_RED_TAG16_SCORING =
        new Translation2d(Units.inchesToMeters(609.465), Units.inchesToMeters(140.720));

    /** Blue Tag 31, Audience side. Original: (42.075", 134.970") */
    public static final Translation2d ENGAGE_BLUE_TAG31_AUDIENCE =
        new Translation2d(Units.inchesToMeters(42.075), Units.inchesToMeters(134.970));

    /** Blue Tag 31, Scoring side. Original: (42.075", 144.720") */
    public static final Translation2d ENGAGE_BLUE_TAG31_SCORING =
        new Translation2d(Units.inchesToMeters(42.075), Units.inchesToMeters(144.720));

    /** Blue Tag 32, Audience side. Original: (42.075", 167.220") */
    public static final Translation2d ENGAGE_BLUE_TAG32_AUDIENCE =
        new Translation2d(Units.inchesToMeters(42.075), Units.inchesToMeters(167.220));

    /** Blue Tag 32, Scoring side. Original: (42.075", 177.220") */
    public static final Translation2d ENGAGE_BLUE_TAG32_SCORING =
        new Translation2d(Units.inchesToMeters(42.075), Units.inchesToMeters(177.220));

    // -------------------------------------------------------------------------
    // CLIMBER GEOMETRY — PLACEHOLDERS, measure from CAD before competition
    // -------------------------------------------------------------------------

    /**
     * Translation from robot CENTER to climber O CENTER, in robot frame.
     * Back-mounted: X is negative (behind robot center), Y near zero.
     * PLACEHOLDER — measure from CAD.
     * Original: (-12.0", 0.0")
     */
    public static final Translation2d CLIMBER_OFFSET_FROM_ROBOT_CENTER =
        new Translation2d(Units.inchesToMeters(-12.0), 0.0);

    /**
     * Climber motor position for O at Level 1 rung height (27" + clearance).
     * Motor zeros at upper hard stop (0.0). Negative = below hard stop = O extended.
     * PLACEHOLDER — jog climber to L1 rung height and read Climb/Position on SmartDashboard.
     */
    public static final double CLIMBER_RUNG_HEIGHT_ROTATIONS = -5.0; // PLACEHOLDER

    /**
     * Climber motor position for stowed / safe travel.
     * Near upper hard stop but with margin to avoid fighting limit.
     * PLACEHOLDER — verify on robot.
     */
    public static final double CLIMBER_STOW_ROTATIONS = -0.3; // PLACEHOLDER

    /**
     * Climber motor position at lower limit (fully retracted to lift robot).
     * Matches existing getReady()/climb() target from legacy Climb code.
     */
    public static final double CLIMBER_LIFT_ROTATIONS = -9.2;

    /**
     * Position tolerance for "at rung height" check (motor rotations).
     * PLACEHOLDER — tighten after CLIMBER_KP is tuned.
     */
    public static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS = 0.5;

    // -------------------------------------------------------------------------
    // CLIMBER GAINS — PLACEHOLDER, tune with Phoenix Tuner X on robot
    // -------------------------------------------------------------------------

    /** Slot 0 kP for PositionVoltage control (volts/rotation). PLACEHOLDER. */
    public static final double CLIMBER_KP = 2.0;

    // -------------------------------------------------------------------------
    // SOFT LIMITS (motor rotations) — matches calibrated encoder convention
    // -------------------------------------------------------------------------

    /** Forward (positive / upward) soft limit. Slightly above zero to account for encoder drift. */
    public static final double SOFT_LIMIT_FORWARD_ROTATIONS = 0.5;

    /** Reverse (negative / downward) soft limit. Slightly below lower hard stop. */
    public static final double SOFT_LIMIT_REVERSE_ROTATIONS = -9.5;

    // -------------------------------------------------------------------------
    // ALIGNMENT TOLERANCES
    // -------------------------------------------------------------------------

    /** X alignment tolerance (meters). Tight — rung must align with O opening. */
    public static final double X_TOLERANCE_M = 0.02;

    /** Y alignment tolerance (meters). Loose — O captures rung anywhere along its length. */
    public static final double Y_TOLERANCE_M = 0.05;

    /** Heading alignment tolerance (radians). ~1°. */
    public static final double HEADING_TOLERANCE_RAD = Math.toRadians(1.0);

    /** Duration tolerance must be held continuously before ENGAGING begins (seconds). */
    public static final double TOLERANCE_DEBOUNCE_S = 0.25;

    // -------------------------------------------------------------------------
    // STATE TIMEOUTS (seconds)
    // -------------------------------------------------------------------------

    public static final double PATHFINDING_TIMEOUT_S    = 4.0;
    public static final double FINAL_APPROACH_TIMEOUT_S = 3.0;
    /** Extra time given to auto routines before accepting current pose and engaging anyway. */
    public static final double AUTO_FALLBACK_EXTRA_S    = 0.5;
    public static final double ALIGNED_HOLD_TIMEOUT_S   = 1.0;
    public static final double LIFTING_TIMEOUT_S        = 5.0;

    // -------------------------------------------------------------------------
    // APPROACH
    // -------------------------------------------------------------------------

    /** Standoff distance from capture pose along approach vector (meters). */
    public static final double STANDOFF_DISTANCE_M = 1.5;

    /**
     * Distance threshold at which localization switches from MegaTag2 to single-tag
     * relative pose (meters).
     */
    public static final double SINGLE_TAG_SWITCH_DISTANCE_M = 1.5;

    /** Time without seeing target tag before falling back to MegaTag2 (seconds). */
    public static final double SINGLE_TAG_TIMEOUT_S = 0.5;

    /**
     * Standard deviation for single-tag final approach vision measurement (meters).
     * Tighter than MegaTag2 since we're using one known reference tag at close range.
     */
    public static final double SINGLE_TAG_XY_STD_DEV = 0.05;

    // -------------------------------------------------------------------------
    // PATHFINDER CONSTRAINTS (60% of max speed/acceleration)
    // -------------------------------------------------------------------------

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.6,
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.6, // approx max accel
        DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.6,
        DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.6
    );

    // -------------------------------------------------------------------------
    // FINAL APPROACH PID
    // -------------------------------------------------------------------------

    /** ProfiledPIDController kP for X and Y translation (m/s per meter). */
    public static final double TRANSLATION_KP = 4.0;

    /** ProfiledPIDController max translation speed for final approach (m/s). */
    public static final double APPROACH_MAX_SPEED_MPS =
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.4;

    /** ProfiledPIDController max translation acceleration for final approach (m/s²). */
    public static final double APPROACH_MAX_ACCEL_MPSS =
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.4;

    // -------------------------------------------------------------------------
    // ENGAGING STATE
    // -------------------------------------------------------------------------

    /** Backward velocity during ENGAGING (robot frame -X, m/s). */
    public static final double ENGAGE_VELOCITY_MPS = 0.15;

    /** Distance to travel backward during ENGAGING before transitioning to LIFTING (meters). */
    public static final double ENGAGE_DISTANCE_M = 0.10;

    /** Hard timeout for ENGAGING state (seconds). Timeout → ABORTED, not LIFTING. */
    public static final double ENGAGE_TIMEOUT_S = 2.0;

    /**
     * Average drivetrain drive motor stator current threshold for stall detection (amps).
     * When drive motors stall against the tower, current spikes above this value.
     */
    public static final double ENGAGE_CURRENT_SPIKE_A = 30.0;

    /**
     * Duration current must exceed ENGAGE_CURRENT_SPIKE_A to trigger stall detection (seconds).
     * Filters out transient spikes from acceleration.
     */
    public static final double ENGAGE_CURRENT_SPIKE_DURATION_S = 0.1;

    // -------------------------------------------------------------------------
    // LED HOOK (no LED subsystem exists; placeholder for future wiring)
    // -------------------------------------------------------------------------
    // When an LED subsystem is added, connect these patterns:
    //   IDLE / ABORTED:      solid off or red
    //   PATHFINDING:         breathing yellow
    //   FINAL_APPROACH:      fast yellow blink
    //   ALIGNED_HOLD:        solid green (LARGE VISUAL CUE — driver cannot see climber)
    //   ENGAGING:            strobing white
    //   LIFTING:             strobing blue
    //   CLIMBED:             solid blue

    // -------------------------------------------------------------------------
    // DRIVER BINDING CONSTANTS — assign physical buttons to these
    // -------------------------------------------------------------------------

    // Autoclimb trigger: driver controller X button (configured in RobotContainer)
    // Engagement target cycle button: PLACEHOLDER — assign in RobotContainer
    // Reset heading: driver controller Start (Menu/three-lines) button
}
