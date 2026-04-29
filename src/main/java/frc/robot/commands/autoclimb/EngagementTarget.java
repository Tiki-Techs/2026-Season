package frc.robot.commands.autoclimb;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum of all 8 engagement targets (4 per alliance).
 *
 * Each target encodes:
 *   - engagementPoint: field position where the climber must be placed (meters)
 *   - nearestTagId:    AprilTag to use for single-tag final-approach localization
 *   - targetHeading:   WPILib robot heading at engagement (back faces tower wall)
 *   - isRed:           which alliance this target belongs to
 *   - approachYSign:   +1 = standoff is at +Y of capture (robot slides in -Y during ENGAGE)
 *                      -1 = standoff is at -Y of capture (robot slides in +Y during ENGAGE)
 *
 * HEADING CONVENTION (WPILib, CCW positive, 0° = +X field direction):
 *   Red targets:  180° — robot front toward field center (-X), back toward Red wall (+X)
 *   Blue targets:   0° — robot front toward field center (+X), back toward Blue wall (-X)
 *
 * APPROACH DIRECTION:
 *   The N-shape climber threads onto the pipe from the rung END (±Y direction).
 *   standoffPose is 1.5m alongside the rung end in the ±Y direction from capturePose.
 *   During ENGAGE the robot slides in ±Y (parallel to driver station wall) to thread the pipe.
 *
 * capturePose  = robot center pose when climber is at engagementPoint
 * standoffPose = approach pose 1.5m outside the rung end (±Y from capturePose)
 */
public enum EngagementTarget {

    // ---- RED ALLIANCE -------------------------------------------------------
    // For Red: Audience is +Y side of upright, Scoring is -Y side.
    // Standoff is further out from rung end (away from upright center).

    /** Red tower, Tag 15 side, Audience (+Y) rung end. Approach from +Y. */
    RED_TAG15_AUDIENCE(AutoclimbConstants.ENGAGE_RED_TAG15_AUDIENCE, 15,
        Rotation2d.fromDegrees(180), true, +1),

    /** Red tower, Tag 15 side, Scoring (-Y) rung end. Approach from -Y. */
    RED_TAG15_SCORING(AutoclimbConstants.ENGAGE_RED_TAG15_SCORING, 15,
        Rotation2d.fromDegrees(180), true, -1),

    /** Red tower, Tag 16 side, Audience (+Y) rung end. Approach from +Y. */
    RED_TAG16_AUDIENCE(AutoclimbConstants.ENGAGE_RED_TAG16_AUDIENCE, 16,
        Rotation2d.fromDegrees(180), true, +1),

    /** Red tower, Tag 16 side, Scoring (-Y) rung end. Approach from -Y. */
    RED_TAG16_SCORING(AutoclimbConstants.ENGAGE_RED_TAG16_SCORING, 16,
        Rotation2d.fromDegrees(180), true, -1),

    // ---- BLUE ALLIANCE ------------------------------------------------------
    // For Blue: Audience is -Y side of upright, Scoring is +Y side.

    /** Blue tower, Tag 31 side, Audience (-Y) rung end. Approach from -Y. */
    BLUE_TAG31_AUDIENCE(AutoclimbConstants.ENGAGE_BLUE_TAG31_AUDIENCE, 31,
        Rotation2d.fromDegrees(0), false, -1),

    /** Blue tower, Tag 31 side, Scoring (+Y) rung end. Approach from +Y. */
    BLUE_TAG31_SCORING(AutoclimbConstants.ENGAGE_BLUE_TAG31_SCORING, 31,
        Rotation2d.fromDegrees(0), false, +1),

    /** Blue tower, Tag 32 side, Audience (-Y) rung end. Approach from -Y. */
    BLUE_TAG32_AUDIENCE(AutoclimbConstants.ENGAGE_BLUE_TAG32_AUDIENCE, 32,
        Rotation2d.fromDegrees(0), false, -1),

    /** Blue tower, Tag 32 side, Scoring (+Y) rung end. Approach from +Y. */
    BLUE_TAG32_SCORING(AutoclimbConstants.ENGAGE_BLUE_TAG32_SCORING, 32,
        Rotation2d.fromDegrees(0), false, +1);

    // -------------------------------------------------------------------------

    public final Translation2d engagementPoint;
    public final int nearestTagId;
    public final Rotation2d targetHeading;
    public final boolean isRed;
    /**
     * Sign of the Y-axis approach direction.
     * +1 = standoff is at capturePose.Y + STANDOFF_DISTANCE; ENGAGE drives in -Y.
     * -1 = standoff is at capturePose.Y - STANDOFF_DISTANCE; ENGAGE drives in +Y.
     */
    public final int approachYSign;

    EngagementTarget(Translation2d engagementPoint, int nearestTagId,
                     Rotation2d targetHeading, boolean isRed, int approachYSign) {
        this.engagementPoint = engagementPoint;
        this.nearestTagId    = nearestTagId;
        this.targetHeading   = targetHeading;
        this.isRed           = isRed;
        this.approachYSign   = approachYSign;
    }

    /**
     * Computes the robot center pose such that the climber is at {@link #engagementPoint}.
     *
     * capturePose = engagementPoint - CLIMBER_OFFSET.rotateBy(heading)
     *
     * The climber offset is negative X in robot frame (behind center).
     * Rotating by heading converts it to field frame, then subtracting from
     * the engagement point gives the robot center that places the climber at the engagement point.
     */
    public Pose2d getCapturePose() {
        Translation2d offsetInField =
            AutoclimbConstants.CLIMBER_OFFSET_FROM_ROBOT_CENTER.rotateBy(targetHeading);
        Translation2d robotCenter = engagementPoint.minus(offsetInField);
        return new Pose2d(robotCenter, targetHeading);
    }

    /**
     * Computes the standoff pose 1.5m outside the rung end in the ±Y direction.
     *
     * The robot approaches from the side (parallel to the driver station wall) so the
     * pipe slides into the N-shape opening from the rung end. standoffPose is positioned
     * at the same X as capturePose but offset in Y by approachYSign * STANDOFF_DISTANCE.
     */
    public Pose2d getStandoffPose() {
        Pose2d capture = getCapturePose();
        Translation2d standoffPos = capture.getTranslation().plus(
            new Translation2d(0, approachYSign * AutoclimbConstants.STANDOFF_DISTANCE_M));
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
