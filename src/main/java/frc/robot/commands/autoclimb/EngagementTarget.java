package frc.robot.commands.autoclimb;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum of all 8 Level 1 engagement targets (4 per alliance).
 *
 * Each target encodes:
 *   - engagementPoint: field position where the climber O must be placed (meters)
 *   - nearestTagId:    AprilTag to use for single-tag final-approach localization
 *   - targetHeading:   WPILib robot heading at engagement (back faces tower)
 *   - isRed:           which alliance this target belongs to
 *
 * HEADING CONVENTION (WPILib, CCW positive, 0° = +X field direction):
 *   Red targets:  180° — robot front toward field center (-X), back toward Red wall (+X)
 *   Blue targets:   0° — robot front toward field center (+X), back toward Blue wall (-X)
 *
 * capturePose  = robot center pose when climber O is at engagementPoint
 * standoffPose = approach pose 1.5m toward field center from capturePose
 */
public enum EngagementTarget {

    // ---- RED ALLIANCE -------------------------------------------------------
    /** Red tower, Tag 15 side, Audience (+Y) rung end. */
    RED_TAG15_AUDIENCE(AutoclimbConstants.ENGAGE_RED_TAG15_AUDIENCE, 15,
        Rotation2d.fromDegrees(180), true),

    /** Red tower, Tag 15 side, Scoring (-Y) rung end. */
    RED_TAG15_SCORING(AutoclimbConstants.ENGAGE_RED_TAG15_SCORING, 15,
        Rotation2d.fromDegrees(180), true),

    /** Red tower, Tag 16 side, Audience (+Y) rung end. */
    RED_TAG16_AUDIENCE(AutoclimbConstants.ENGAGE_RED_TAG16_AUDIENCE, 16,
        Rotation2d.fromDegrees(180), true),

    /** Red tower, Tag 16 side, Scoring (-Y) rung end. */
    RED_TAG16_SCORING(AutoclimbConstants.ENGAGE_RED_TAG16_SCORING, 16,
        Rotation2d.fromDegrees(180), true),

    // ---- BLUE ALLIANCE ------------------------------------------------------
    /** Blue tower, Tag 31 side, Audience (+Y ... reflected) rung end. */
    BLUE_TAG31_AUDIENCE(AutoclimbConstants.ENGAGE_BLUE_TAG31_AUDIENCE, 31,
        Rotation2d.fromDegrees(0), false),

    /** Blue tower, Tag 31 side, Scoring rung end. */
    BLUE_TAG31_SCORING(AutoclimbConstants.ENGAGE_BLUE_TAG31_SCORING, 31,
        Rotation2d.fromDegrees(0), false),

    /** Blue tower, Tag 32 side, Audience rung end. */
    BLUE_TAG32_AUDIENCE(AutoclimbConstants.ENGAGE_BLUE_TAG32_AUDIENCE, 32,
        Rotation2d.fromDegrees(0), false),

    /** Blue tower, Tag 32 side, Scoring rung end. */
    BLUE_TAG32_SCORING(AutoclimbConstants.ENGAGE_BLUE_TAG32_SCORING, 32,
        Rotation2d.fromDegrees(0), false);

    // -------------------------------------------------------------------------

    public final Translation2d engagementPoint;
    public final int nearestTagId;
    public final Rotation2d targetHeading;
    public final boolean isRed;

    EngagementTarget(Translation2d engagementPoint, int nearestTagId,
                     Rotation2d targetHeading, boolean isRed) {
        this.engagementPoint = engagementPoint;
        this.nearestTagId    = nearestTagId;
        this.targetHeading   = targetHeading;
        this.isRed           = isRed;
    }

    /**
     * Computes the robot center pose such that the climber O is at {@link #engagementPoint}.
     *
     * capturePose = engagementPoint - CLIMBER_OFFSET.rotateBy(heading)
     *
     * The climber offset is negative X in robot frame (behind center).
     * Rotating by heading converts it to field frame, then subtracting from
     * the engagement point gives the robot center that places the O at the engagement point.
     */
    public Pose2d getCapturePose() {
        Translation2d offsetInField =
            AutoclimbConstants.CLIMBER_OFFSET_FROM_ROBOT_CENTER.rotateBy(targetHeading);
        Translation2d robotCenter = engagementPoint.minus(offsetInField);
        return new Pose2d(robotCenter, targetHeading);
    }

    /**
     * Computes the standoff pose 1.5m toward the field center along the robot's forward direction.
     *
     * The robot's forward direction (robot +X) in field coordinates:
     *   Red  (180°): (-1, 0) — toward Blue / field center
     *   Blue (  0°): (+1, 0) — toward Red  / field center
     *
     * Standoff = capturePose + 1.5m * robotForwardUnitVector
     */
    public Pose2d getStandoffPose() {
        Pose2d capture = getCapturePose();
        // Robot forward unit vector in field coordinates
        Translation2d forward = new Translation2d(
            AutoclimbConstants.STANDOFF_DISTANCE_M, targetHeading);
        Translation2d standoffPos = capture.getTranslation().plus(forward);
        return new Pose2d(standoffPos, targetHeading);
    }

    /**
     * Returns the display name for dashboard publishing.
     */
    public String getDisplayName() {
        return name().replace('_', ' ');
    }

    /**
     * Returns an ordered list of preferred targets for the given alliance.
     * Inboard (scoring-side) targets listed first for better approach clearance.
     *
     * Returns Blue targets if alliance is unknown or null.
     */
    public static List<EngagementTarget> getPreferredTargets(Alliance alliance) {
        if (alliance == Alliance.Red) {
            return List.of(
                RED_TAG15_SCORING,
                RED_TAG15_AUDIENCE,
                RED_TAG16_SCORING,
                RED_TAG16_AUDIENCE
            );
        } else {
            return List.of(
                BLUE_TAG31_SCORING,
                BLUE_TAG31_AUDIENCE,
                BLUE_TAG32_SCORING,
                BLUE_TAG32_AUDIENCE
            );
        }
    }

    /**
     * Returns the default (first preferred) target for the given alliance.
     * Returns BLUE_TAG31_SCORING if alliance is unknown.
     */
    public static EngagementTarget getDefault(Alliance alliance) {
        if (alliance == Alliance.Red) return RED_TAG15_SCORING;
        return BLUE_TAG31_SCORING;
    }
}
