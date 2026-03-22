package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldAiming;
import frc.robot.LimelightHelpers;

/** Handles Limelight camera processing, pose estimation, and target tracking. */
public class Vision extends SubsystemBase {

    private final SwerveSubsystem drivetrain;
    private final StructPublisher<Pose2d> posePublisher;
    private final PIDController m_aimController;

    // Cached Limelight state - updated once per loop in periodic()
    private boolean cachedTV = false;
    private double cachedTX = 0.0;
    private double cachedTagDist = 0.0;

    // Climb limelight specific cache for aim control
    private boolean cachedClimbTV = false;
    private double cachedClimbTX = 0.0;

    private double lastTargetAngle = 0.0;

    public Vision(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        m_aimController = new PIDController(3.5, 0.8, 0.2);  // Faster, stronger integral
        m_aimController.enableContinuousInput(-Math.PI, Math.PI);
        m_aimController.setTolerance(Math.toRadians(1.0)); // 1° tolerance
        m_aimController.setIZone(Math.toRadians(10.0)); // Integrate when within 10°
    }

    /** Calculates forward velocity for ranging to a target using cached tag distance. */
    public double limelight_range_proportional() {
        double kP = 0.4;

        if (!cachedTV || cachedTagDist == 0.0) return 0.0;

        double error = cachedTagDist - VisionConstants.TARGET_DISTANCE_METERS;
        if (Math.abs(error) < 0.1) return 0.0;

        return error * kP;
    }

    /** Calculates angular velocity for aiming at a target using proportional control (climb limelight only). */
    public double limelight_aim_proportional() {
        double kP = 1.5;
        double deadzone = 0.5; // degrees - tight deadzone for accuracy

        if (!cachedClimbTV) return 0.0;

        // Apply pigeon offset to account for off-center mounting
        double adjustedTX = cachedClimbTX + VisionConstants.PIGEON_ANGLE_OFFSET_DEGREES;

        if (Math.abs(adjustedTX) < deadzone) return 0.0;

        double txRadians = Math.toRadians(adjustedTX);
        return txRadians * kP;
    }

    /** Gets the cached distance to the nearest AprilTag in meters. */
    public double getTagDistance() {
        return cachedTagDist;
    }

    /** Returns whether a valid target is currently visible. */
    public boolean hasTarget() {
        return cachedTV;
    }

    private double getGoalX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return VisionConstants.RED_GOAL_X_METERS;
        }
        return VisionConstants.BLUE_GOAL_X_METERS;
    }

    private double getGoalY() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return VisionConstants.RED_GOAL_Y_METERS;
        }
        return VisionConstants.BLUE_GOAL_Y_METERS;
    }

    /** Calculates the distance from the robot's current position to the goal. */
    public double getDistanceToGoal() {
        Pose2d currentPose = drivetrain.getState().Pose;
        // Use FieldGeomUtils for consistency
        return FieldAiming.getDistanceToHub(currentPose);
    }

    /** Checks if the robot is within shooting range (not in the neutral zone). */
    public boolean isInShootingRange() {
        Pose2d currentPose = drivetrain.getState().Pose;
        // Use FieldGeomUtils for consistency
        return FieldAiming.isInScoringRange(currentPose);
    }

    /** Returns the target rotation to face the goal or alliance wall. */
    public Rotation2d getTargetRotation() {
        Pose2d currentPose = drivetrain.getState().Pose;

        if (isInShootingRange()) {
            // Use FieldGeomUtils for consistency
            return FieldAiming.getAngleToHub(currentPose);
        } else {
            var alliance = DriverStation.getAlliance();
            return (alliance.isPresent() && alliance.get() == Alliance.Red)
                ? new Rotation2d(0)
                : new Rotation2d(Math.PI);
        }
    }

    /** Calculates angular velocity to rotate toward the goal or alliance wall (odometry-based). */
    public double getRotationToGoal() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double targetAngle = getTargetRotation().getRadians();

        // Reset controller if target changed (switching between hub/wall)
        if (Math.abs(targetAngle - lastTargetAngle) > 0.1) {
            m_aimController.reset();
            lastTargetAngle = targetAngle;
        }

        if (m_aimController.atSetpoint()) {
            return 0.0;
        }

        return m_aimController.calculate(currentPose.getRotation().getRadians(), targetAngle);
    }

    /** Calculates angular velocity using TX from Limelight (direct camera feedback, no odometry). */
    public double getRotationToGoalTX() {
        if (!cachedTV) return 0.0;

        // TX is already the angular error to the tag - use it directly
        double txRadians = Math.toRadians(cachedTX);

        if (Math.abs(cachedTX) < 0.5) {  // 0.5 degree dead zone
            m_aimController.reset();
            return 0.0;
        }

        // PID with TX as measurement, 0 as setpoint
        return m_aimController.calculate(txRadians, 0.0);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = drivetrain.getState().Pose;
        posePublisher.set(currentPose);

        cachedTV = false;
        cachedTX = 0.0;
        cachedTagDist = 0.0;
        cachedClimbTV = false;
        cachedClimbTX = 0.0;
        double bestTagDist = Double.MAX_VALUE;

        // For averaging TX from multiple hub tags
        double txSum = 0.0;
        int hubTagCount = 0;

        // For climb limelight TX averaging
        double climbTxSum = 0.0;
        int climbHubTagCount = 0;

        // Get hub tag IDs for current alliance (for TX-based aiming)
        var alliance = DriverStation.getAlliance();
        int[] hubTagIds = (alliance.isPresent() && alliance.get() == Alliance.Red)
            ? VisionConstants.RED_HUB_TAG_IDS
            : VisionConstants.BLUE_HUB_TAG_IDS;

        boolean rejectAllUpdates = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720;

        for (int i = 0; i < VisionConstants.ALL_LIMELIGHTS.length; i++) {
            String limelightName = VisionConstants.ALL_LIMELIGHTS[i];

            boolean tv = LimelightHelpers.getTV(limelightName);
            double tx = LimelightHelpers.getTX(limelightName);
            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);

            LimelightHelpers.SetRobotOrientation(
                limelightName,
                currentPose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            SmartDashboard.putBoolean("Vision/" + limelightName + "/TV", tv);
            SmartDashboard.putNumber("Vision/" + limelightName + "/TX", tx);
            SmartDashboard.putNumber("Vision/" + limelightName + "/TagID", tagId);

            if (mt2 != null && mt2.tagCount >= 1) {
                SmartDashboard.putNumber("Vision/" + limelightName + "/TagDist", mt2.avgTagDist);
                SmartDashboard.putNumber("Vision/" + limelightName + "/TagCount", mt2.tagCount);
                SmartDashboard.putNumber("Vision/" + limelightName + "/PoseX", mt2.pose.getX());
                SmartDashboard.putNumber("Vision/" + limelightName + "/PoseY", mt2.pose.getY());

                // Reject poses outside the field (bad data from transitional frames)
                double poseX = mt2.pose.getX();
                double poseY = mt2.pose.getY();
                if (poseX < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseX > VisionConstants.FIELD_LENGTH_METERS + VisionConstants.FIELD_BORDER_MARGIN
                    || poseY < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseY > VisionConstants.FIELD_WIDTH_METERS + VisionConstants.FIELD_BORDER_MARGIN) {
                    continue;
                }

                // Check if this is a hub tag for TX-based aiming
                boolean isHubTag = false;
                for (int id : hubTagIds) {
                    if (tagId == id) {
                        isHubTag = true;
                        break;
                    }
                }

                // Accumulate TX from all visible hub tags to aim at center
                if (tv && isHubTag) {
                    txSum += tx;
                    hubTagCount++;

                    // Track climb limelight TX separately for aim control
                    if (limelightName.equals(VisionConstants.LIMELIGHT_CLIMB)) {
                        climbTxSum += tx;
                        climbHubTagCount++;
                    }

                    // Track closest tag distance
                    if (mt2.avgTagDist < bestTagDist) {
                        bestTagDist = mt2.avgTagDist;
                        cachedTagDist = mt2.avgTagDist;
                    }
                }

                if (!rejectAllUpdates) {
                    // Higher stdDev = less trust in vision, more reliance on odometry
                    // Floor at 0.5 to prevent over-trusting vision at close range
                    double xyStdDev = Math.max(0.5, 0.7 * mt2.avgTagDist / mt2.tagCount);

                    SmartDashboard.putNumber("Vision/" + limelightName + "/StdDev", xyStdDev);
                    SmartDashboard.putBoolean("Vision/" + limelightName + "/PoseAccepted", true);

                    drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
                } else {
                    SmartDashboard.putBoolean("Vision/" + limelightName + "/PoseAccepted", false);
                    SmartDashboard.putString("Vision/" + limelightName + "/RejectReason", "High angular velocity");
                }
            }
        }

        // Average TX from all hub tags to aim at hub center
        if (hubTagCount > 0) {
            cachedTV = true;
            cachedTX = txSum / hubTagCount;
        }

        // Average TX from climb limelight hub tags for aim control
        if (climbHubTagCount > 0) {
            cachedClimbTV = true;
            cachedClimbTX = climbTxSum / climbHubTagCount;
        }

        SmartDashboard.putNumber("Vision/DistanceToGoal", getDistanceToGoal());
        SmartDashboard.putBoolean("Vision/InShootingRange", isInShootingRange());
        SmartDashboard.putNumber("Vision/RotationToGoal", getRotationToGoal());
        SmartDashboard.putBoolean("Vision/ClimbLimelightTV", cachedClimbTV);
        SmartDashboard.putNumber("Vision/ClimbLimelightTX", cachedClimbTX);
        SmartDashboard.putNumber("Vision/ClimbTXWithOffset", cachedClimbTX + VisionConstants.PIGEON_ANGLE_OFFSET_DEGREES);
        SmartDashboard.putNumber("Vision/ClimbAimOutput", limelight_aim_proportional());

        // Debug info for auto-align
        Pose2d pose = drivetrain.getState().Pose;
        double targetAngle = isInShootingRange()
            ? Math.atan2(getGoalY() - pose.getY(), getGoalX() - pose.getX())
            : (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? 0 : Math.PI);
        double currentHeading = pose.getRotation().getRadians();
        double error = targetAngle - currentHeading;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        SmartDashboard.putNumber("Vision/TargetAngleDeg", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Vision/CurrentHeadingDeg", Math.toDegrees(currentHeading));
        SmartDashboard.putNumber("Vision/AngleErrorDeg", Math.toDegrees(error));
        SmartDashboard.putNumber("Vision/PIDOutput", getRotationToGoal());
        SmartDashboard.putBoolean("Vision/PIDAtSetpoint", m_aimController.atSetpoint());
    }
}
