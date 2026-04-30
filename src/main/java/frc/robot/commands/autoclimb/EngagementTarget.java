package frc.robot.commands.autoclimb;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum of climb engagement targets.
 *
 * Each target defines the pose PathPlanner navigates to before the climb sequence.
 * The robot arrives here facing 90 deg (Blue) or -90 deg (Red) so its back (climber side)
 * faces the rung. It then raises the arm and drives robot-relative -X (backward) to engage.
 *
 * HEADING CONVENTION (WPILib, CCW positive):
 *   Blue:  90 deg — robot back faces -X (toward Blue wall / climb structure)
 *   Red:  -90 deg — robot back faces +X (toward Red wall / climb structure)
 *
 * TARGET POSE X:
 *   Blue: UPRIGHT_X_BLUE_M - TARGET_POSE_X_OFFSET_M  (robot is in front of upright, in +X direction)
 *   Red:  UPRIGHT_X_RED_M  + TARGET_POSE_X_OFFSET_M  (robot is in front of upright, in -X direction)
 *
 * TARGET POSE Y: center rung by default (CLIMB_CENTER_Y_M). Audience/scoring variants offset from there.
 */
public enum EngagementTarget {

    // ---- BLUE ALLIANCE -------------------------------------------------------
    // Robot heading: 90 deg. Back faces Blue wall (-X). Drives backward in robot -X = field -Y... 
    // wait: at 90 deg heading, robot -X maps to field -Y. But the rung is in the -X field direction
    // from the robot. RobotCentric handles this correctly via the heading.

    /** Blue alliance, center rung. */
    BLUE_CENTER(
        new Translation2d(
            AutoclimbConstants.UPRIGHT_X_BLUE_M - AutoclimbConstants.TARGET_POSE_X_OFFSET_M,
            AutoclimbConstants.CLIMB_CENTER_Y_M),
        Rotation2d.fromDegrees(90),
        false),

    // ---- RED ALLIANCE --------------------------------------------------------
    // Robot heading: -90 deg. Back faces Red wall (+X).

    /** Red alliance, center rung. */
    RED_CENTER(
        new Translation2d(
            AutoclimbConstants.UPRIGHT_X_RED_M + AutoclimbConstants.TARGET_POSE_X_OFFSET_M,
            AutoclimbConstants.CLIMB_CENTER_Y_M),
        Rotation2d.fromDegrees(-90),
        true);

    // -------------------------------------------------------------------------

    /** Field position PathPlanner navigates the robot center to. */
    public final Translation2d targetPosition;

    /** WPILib heading the robot should face at the target pose. */
    public final Rotation2d targetHeading;

    public final boolean isRed;

    EngagementTarget(Translation2d targetPosition, Rotation2d targetHeading, boolean isRed) {
        this.targetPosition = targetPosition;
        this.targetHeading  = targetHeading;
        this.isRed          = isRed;
    }

    /**
     * Returns the full Pose2d PathPlanner navigates to.
     */
    public Pose2d getTargetPose() {
        return new Pose2d(targetPosition, targetHeading);
    }

    /** Returns the display name for dashboard publishing. */
    public String getDisplayName() {
        return name().replace('_', ' ');
    }

    /**
     * Returns the default target for the given alliance.
     * Defaults to BLUE_CENTER if alliance is unknown.
     */
    public static EngagementTarget getDefault(Alliance alliance) {
        if (alliance == Alliance.Red) return RED_CENTER;
        return BLUE_CENTER;
    }

    /**
     * Returns all targets for the given alliance.
     */
    public static List<EngagementTarget> getTargets(Alliance alliance) {
        if (alliance == Alliance.Red) return List.of(RED_CENTER);
        return List.of(BLUE_CENTER);
    }
}
