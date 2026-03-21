package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/** Handles Limelight camera processing, pose estimation, and target tracking. */
public class Vision extends SubsystemBase {

    private final SwerveSubsystem drivetrain;
    private final StructPublisher<Pose2d> posePublisher;
    private Climb climb = null;

    // Cached Limelight state - updated once per loop in periodic()
    private boolean cachedTV = false;
    private double cachedTX = 0.0;
    private double cachedTagDist = 0.0;

    private double lastRotationError = 0.0;

    public Vision(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();
    }

    /** Sets the climb reference for dynamic camera pose updates. */
    public void setClimb(Climb climb) {
        this.climb = climb;
    }

    /** Calculates forward velocity for ranging to a target using cached tag distance. */
    public double limelight_range_proportional() {
        double kP = 0.4;

        if (!cachedTV || cachedTagDist == 0.0) return 0.0;

        double error = cachedTagDist - VisionConstants.TARGET_DISTANCE_METERS;
        if (Math.abs(error) < 0.1) return 0.0;

        return error * kP;
    }

    /** Calculates angular velocity for aiming at a target using proportional control. */
    public double limelight_aim_proportional() {
        double kP = 1.5;
     double minOutput = 0.15;

        if (!cachedTV) return 0.0;
        if (Math.abs(cachedTX) < 5.0) return 0.0;

        double txRadians = Math.toRadians(cachedTX);
        double output = txRadians * kP;

        if (output > 0) output = Math.max(output, minOutput);
        else output = Math.min(output, -minOutput);

        return output;
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
        double dx = getGoalX() - currentPose.getX();
        double dy = getGoalY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Checks if the robot is within shooting range (not in the neutral zone). */
    public boolean isInShootingRange() {
        double robotX = drivetrain.getState().Pose.getX();
        return robotX <= VisionConstants.BLUE_GOAL_X_METERS || robotX >= VisionConstants.RED_GOAL_X_METERS;
    }

    /** Calculates angular velocity to rotate toward the goal or alliance wall. */
    public double getRotationToGoal() {
        double kP = 3.0;
        double kD = 0.2;
        double minOutput = 0.2;

        Pose2d currentPose = drivetrain.getState().Pose;
        double targetAngle;

        if (isInShootingRange()) {
            double dx = getGoalX() - currentPose.getX();
            double dy = getGoalY() - currentPose.getY();
            targetAngle = Math.atan2(dy, dx);
        } else {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                targetAngle = 0;
            } else {
                targetAngle = Math.PI;
            }
        }

        double currentHeading = currentPose.getRotation().getRadians();
        double error = targetAngle - currentHeading;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        double derivative = (error - lastRotationError) / 0.02;
        lastRotationError = error;

        if (Math.abs(error) < Math.toRadians(1.0)) return 0.0;

        double output = error * kP + derivative * kD;
        if (output > 0) output = Math.max(output, minOutput);
        else output = Math.min(output, -minOutput);

        return output;
    }

    @Override
    public void periodic() {
        Pose2d currentPose = drivetrain.getState().Pose;
        posePublisher.set(currentPose);

        // Update climb-mounted camera pose based on climb position
        // Climb zeros at top (position=0), negative when retracted down
        if (climb != null && climb.isCalibrated()) {
            double climbPosition = climb.getPosition();
            double heightOffset = climbPosition * VisionConstants.CLIMB_METERS_PER_ROTATION;

            LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.LIMELIGHT_CLIMB,
                VisionConstants.CLIMB_CAM_FORWARD,
                VisionConstants.CLIMB_CAM_SIDE,
                VisionConstants.CLIMB_CAM_UP_BASE + heightOffset,
                VisionConstants.CLIMB_CAM_ROLL,
                VisionConstants.CLIMB_CAM_PITCH,
                VisionConstants.CLIMB_CAM_YAW
            );
        }

        cachedTV = false;
        cachedTX = 0.0;
        cachedTagDist = 0.0;
        double bestTagDist = Double.MAX_VALUE;

        boolean rejectAllUpdates = Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720;

        for (int i = 0; i < VisionConstants.ALL_LIMELIGHTS.length; i++) {
            String limelightName = VisionConstants.ALL_LIMELIGHTS[i];

            boolean tv = LimelightHelpers.getTV(limelightName);
            double tx = LimelightHelpers.getTX(limelightName);


            LimelightHelpers.SetRobotOrientation(
                limelightName,
                currentPose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            SmartDashboard.putBoolean("Vision/" + limelightName + "/TV", tv);
            SmartDashboard.putNumber("Vision/" + limelightName + "/TX", tx);

            if (mt2 != null && mt2.tagCount >= 2) {
                SmartDashboard.putNumber("Vision/" + limelightName + "/TagDist", mt2.avgTagDist);

                // Reject poses outside the field (bad data from transitional frames)
                double poseX = mt2.pose.getX();
                double poseY = mt2.pose.getY();
                if (poseX < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseX > VisionConstants.FIELD_LENGTH_METERS + VisionConstants.FIELD_BORDER_MARGIN
                    || poseY < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseY > VisionConstants.FIELD_WIDTH_METERS + VisionConstants.FIELD_BORDER_MARGIN) {
                    continue;
                }

                if (tv && mt2.avgTagDist < bestTagDist) {
                    cachedTV = true;
                    cachedTX = tx;
                    cachedTagDist = mt2.avgTagDist;
                    bestTagDist = mt2.avgTagDist;
                }

                if (!rejectAllUpdates) {
                    // Higher stdDev = less trust in vision, more reliance on odometry
                    // Floor at 0.5 to prevent over-trusting vision at close range
                    double stdDev = Math.max(0.5, 0.7 * mt2.avgTagDist / mt2.tagCount);
                    drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(stdDev, stdDev, 9999999));
                }
            }
        }

        SmartDashboard.putNumber("Vision/DistanceToGoal", getDistanceToGoal());
        SmartDashboard.putBoolean("Vision/InShootingRange", isInShootingRange());
        SmartDashboard.putNumber("Vision/RotationToGoal", getRotationToGoal());
    }
}
