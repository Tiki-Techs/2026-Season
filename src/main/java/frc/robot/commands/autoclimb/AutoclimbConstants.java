package frc.robot.commands.autoclimb;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/**
 * All tunables, field geometry, and tolerances for the autoclimb system.
 *
 * FIELD ORIGIN: Blue alliance wall = X=0, bottom of field = Y=0, +X toward Red wall.
 * All constants are in METERS internally. Original inch values noted in comments.
 *
 * CLIMB SEQUENCE:
 *   1. PATHFINDING:    PathPlanner drives to the target pose (beside the rung,
 *                      robot facing 90 deg Blue / -90 deg Red so the back faces the rung).
 *   2. RAISE_CLIMB:    Arm extends to upper limit switch (above rung height). Drivetrain locked.
 *   3. DRIVE_BACKWARD: Robot drives robot-relative -X (backward) at BACKWARD_DRIVE_SPEED_MPS
 *                      for DRIVE_BACKWARD_TIMEOUT_S seconds, sliding the O over the rung.
 *   4. LOWER_AND_LIFT: Arm lowers slowly at LOWER_CLIMB_SPEED. O settles on rung, robot lifts.
 *   5. CLIMBED:        Drivetrain brakes locked. Motor holds in brake mode.
 *
 * HEADING CONVENTION (WPILib, CCW positive, 0 deg = toward Red wall):
 *   Blue:  90 deg — robot back faces Blue climb structure
 *   Red:  -90 deg — robot back faces Red climb structure
 *
 * "Drive backward" = robot-relative -X. At 90 deg heading that is field -Y.
 * At -90 deg heading that is field +Y. RobotCentric handles both with no sign logic.
 */
public final class AutoclimbConstants {
    private AutoclimbConstants() {}

    // -------------------------------------------------------------------------
    // FIELD GEOMETRY
    // From 2026 field drawings (all inches converted to meters):
    //   Full field length = 651.22" (16.544 m)
    //   Uprights at 115.05" from each alliance wall
    //   Field width = 317.69" (8.069 m), midline = 158.845" (4.035 m)
    // -------------------------------------------------------------------------

    /** Blue alliance climb upright X (meters). Original: 115.05" from Blue wall. */
    public static final double UPRIGHT_X_BLUE_M = Units.inchesToMeters(115.05);

    /** Red alliance climb upright X (meters). Original: 651.22 - 115.05 = 536.17" from Blue wall. */
    public static final double UPRIGHT_X_RED_M  = Units.inchesToMeters(536.17);

    /**
     * Field midline Y (meters). The climb structure sits on the midline.
     * Original: 317.69 / 2 = 158.845"
     * Adjust this Y if you want to target a specific rung position.
     */


    public static final double CLIMB_CENTER_Y_M = Units.inchesToMeters(188.845);

    // -------------------------------------------------------------------------
    // TARGET POSE
    //
    // PathPlanner navigates the robot to a pose in front of the rung with the
    // correct heading. After arriving, the robot raises its arm and drives backward
    // to engage.
    //
    // X: UPRIGHT_X_BLUE_M or UPRIGHT_X_RED_M, offset by TARGET_POSE_X_OFFSET_M
    //    so the robot center is in front of the rung (not crashed into it).
    // Y: CLIMB_CENTER_Y_M (center rung). Adjust per-target in EngagementTarget.
    //
    // TARGET_POSE_X_OFFSET_M: distance the robot center sits in front of the
    //   rung upright along the field X axis when PathPlanner finishes.
    //   The arm is 13.5" behind robot center. Offset must be large enough that
    //   the arm clears the rung horizontally while being raised.
    //   PLACEHOLDER — tune by watching the arm clear the structure in practice.
    // -------------------------------------------------------------------------

    /**
     * Distance from the upright X toward the robot's forward direction (meters).
     * Blue robot approaches from -X side, so target X = UPRIGHT_X_BLUE_M - this value.
     * Red robot approaches from +X side, so target X = UPRIGHT_X_RED_M + this value.
     * PLACEHOLDER — start at 18" and adjust.
     */
    public static final double TARGET_POSE_X_OFFSET_M = Units.inchesToMeters(74.0);



    // -------------------------------------------------------------------------
    // CLIMBER GEOMETRY
    // -------------------------------------------------------------------------

    /**
     * Translation from robot CENTER to climber, robot frame.
     * Negative X = behind robot center. 13.5" measured from CAD.
     */
    public static final Translation2d CLIMBER_OFFSET_FROM_ROBOT_CENTER =
        new Translation2d(Units.inchesToMeters(-13.5), 0.0);

    // -------------------------------------------------------------------------
    // CLIMBER MOTOR CONSTANTS
    // -------------------------------------------------------------------------

    public static final double CLIMBER_RUNG_HEIGHT_ROTATIONS        = -5.0;  // PLACEHOLDER
    public static final double CLIMBER_STOW_ROTATIONS               = -0.3;  // PLACEHOLDER
    public static final double CLIMBER_LIFT_ROTATIONS               = -9.2;
    public static final double CLIMBER_POSITION_TOLERANCE_ROTATIONS =  0.5;
    public static final double CLIMBER_KP                           =  2.0;
    public static final double SOFT_LIMIT_FORWARD_ROTATIONS         =  0.5;
    public static final double SOFT_LIMIT_REVERSE_ROTATIONS         = -9.5;

    // -------------------------------------------------------------------------
    // LOWER-FOR-CLIMB SPEED
    // -------------------------------------------------------------------------

    /**
     * Duty cycle magnitude for the slow lower phase (0.0-1.0).
     * 45% lets the arm descend smoothly. The rung inside the O stalls the motor
     * before it hits the lower limit switch; the 40A current limit protects it.
     */
    public static final double LOWER_CLIMB_SPEED = 0.45;

    // -------------------------------------------------------------------------
    // STATE TIMEOUTS (seconds)
    // -------------------------------------------------------------------------

    public static final double PATHFINDING_TIMEOUT_S    = 8.0;
    public static final double RAISE_CLIMB_TIMEOUT_S    = 8.0;

    /**
     * How long the robot drives backward to slide the O over the rung end (seconds).
     * At 0.2 m/s, 2.0 s = ~0.4 m of travel.
     * TUNE THIS — increase if the O doesn't fully clear the rung, decrease if it overshoots.
     */
    public static final double DRIVE_BACKWARD_TIMEOUT_S = 2.0;

    public static final double LOWER_LIFT_TIMEOUT_S     = 15.0;

    // -------------------------------------------------------------------------
    // DRIVE BACKWARD SPEED
    // -------------------------------------------------------------------------

    /**
     * Robot-centric backward speed during DRIVE_BACKWARD (m/s).
     * Applied as negative robot-X velocity (robot back moves toward rung).
     * Conservative starting point — increase once engagement is confirmed reliable.
     */
    public static final double BACKWARD_DRIVE_SPEED_MPS = 0.2;

    // -------------------------------------------------------------------------
    // PATHFINDER CONSTRAINTS (60% of max)
    // -------------------------------------------------------------------------

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.6,
        DriveConstants.MAX_SPEED_METERS_PER_SECOND * 0.6,
        DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.6,
        DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.6
    );

    // -------------------------------------------------------------------------
    // VISION
    // -------------------------------------------------------------------------

    public static final double SINGLE_TAG_SWITCH_DISTANCE_M = 1.5;
    public static final double SINGLE_TAG_TIMEOUT_S         = 0.5;
    public static final double SINGLE_TAG_XY_STD_DEV        = 0.05;
}
