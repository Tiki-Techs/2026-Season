package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CornerDumpConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Field geometry utilities for the 2026 Reefscape game.
 * Provides calculations for targeting the scoring hubs (coral/algae placement).
 */
public class FieldAiming {

    // Hub positions for each alliance (center of scoring area)
    public static final Pose2d BLUE_HUB = new Pose2d(
        VisionConstants.BLUE_GOAL_X_METERS,
        VisionConstants.BLUE_GOAL_Y_METERS,
        Rotation2d.fromDegrees(0)
    );

    public static final Pose2d RED_HUB = new Pose2d(
        VisionConstants.RED_GOAL_X_METERS,
        VisionConstants.RED_GOAL_Y_METERS,
        Rotation2d.fromDegrees(0)
    );

    /**
     * Helper to get the target hub based on current alliance.
     * @return The target hub pose for the current alliance
     */
    private static Pose2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB;
        }
        return BLUE_HUB;
    }

    /**
     * Calculates the angle to point the FRONT of the robot at the hub.
     * This is the heading the robot should face to shoot at the target.
     *
     * @param robotPose Current robot pose from odometry/pose estimation
     * @return Target rotation (field-relative heading to face the hub)
     */
    public static Rotation2d getAngleToHub(Pose2d robotPose) {
        Translation2d relativeTranslation = getTargetHub().getTranslation().minus(robotPose.getTranslation());
        // getAngle() returns the vector direction from robot to hub
        return relativeTranslation.getAngle();
    }

    /**
     * Calculates the straight-line distance to the target hub in meters.
     * Used for distance-based shooter speed adjustments.
     *
     * @param robotPose Current robot pose from odometry/pose estimation
     * @return Distance to hub in meters
     */
    public static double getDistanceToHub(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(getTargetHub().getTranslation());
    }

    /**
     * Gets the current alliance's hub position.
     *
     * @return The target hub pose
     */
    public static Pose2d getHub() {
        return getTargetHub();
    }

    /**
     * Checks if the robot is in scoring range (outside neutral zone).
     *
     * @param robotPose Current robot pose
     * @return True if robot is in a valid scoring position
     */
    public static boolean isInScoringRange(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // Red alliance shoots from right side of field
            return robotPose.getX() >= VisionConstants.RED_GOAL_X_METERS;
        } else {
            // Blue alliance shoots from left side of field
            return robotPose.getX() <= VisionConstants.BLUE_GOAL_X_METERS;
        }
    }

    // =========================================================================
    // CORNER DUMP TARGETING
    // =========================================================================

    /**
     * Returns the corner target Translation2d for the current alliance and robot Y position.
     *
     * Corner selection logic:
     *   - X: always on your alliance's side of the field (Red wall or Blue wall), inset by CORNER_INSET_X
     *   - Y: audience side corner if robot Y >= FIELD_Y_MIDPOINT, scoring side corner otherwise
     *
     * All four target positions can be tuned via CornerDumpConstants.
     *
     * @param robotPose Current robot pose
     * @return The Translation2d of the selected corner target
     */
    public static Translation2d getCornerTarget(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        double cornerX = isRed
            ? CornerDumpConstants.RED_CORNER_X
            : CornerDumpConstants.BLUE_CORNER_X;

        // Pick audience or scoring side based on which half of the field the robot is on
        double cornerY = (robotPose.getY() >= CornerDumpConstants.FIELD_Y_MIDPOINT)
            ? CornerDumpConstants.AUDIENCE_CORNER_Y
            : CornerDumpConstants.SCORING_CORNER_Y;

        return new Translation2d(cornerX, cornerY);
    }

    /**
     * Calculates the angle the robot should face to shoot toward the selected corner.
     *
     * @param robotPose Current robot pose
     * @return Target heading (field-relative) to aim at the corner
     */
    public static Rotation2d getAngleToCorner(Pose2d robotPose) {
        Translation2d cornerTarget = getCornerTarget(robotPose);
        Translation2d relativeTranslation = cornerTarget.minus(robotPose.getTranslation());
        return relativeTranslation.getAngle();
    }

    /**
     * Calculates straight-line distance from the robot to the selected corner target.
     * Used for distance-based shooter speed selection during corner dumps.
     *
     * @param robotPose Current robot pose
     * @return Distance to corner target in meters
     */
    public static double getDistanceToCorner(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(getCornerTarget(robotPose));
    }

}
