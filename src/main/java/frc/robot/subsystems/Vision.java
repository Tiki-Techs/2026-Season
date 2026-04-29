package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldAiming;
import frc.robot.LimelightHelpers;
import frc.robot.commands.autoclimb.AutoclimbConstants;

/** Handles Limelight camera processing, pose estimation, and target tracking. */
public class Vision extends SubsystemBase {

    /**
     * Controls how pose estimates are fed to the drivetrain pose estimator.
     *
     * FUSED_MEGATAG2: Normal operation — MegaTag2 updates from all limelights.
     * SINGLE_TAG:     Final climb approach — only the one reference tag for the
     *                 selected engagement target is used. MegaTag2 updates are
     *                 suppressed. Falls back to FUSED_MEGATAG2 if the tag is
     *                 not visible for SINGLE_TAG_TIMEOUT_S seconds.
     */
    public enum LocalizationMode { FUSED_MEGATAG2, SINGLE_TAG }

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

    // Localization mode state
    private LocalizationMode m_localizationMode = LocalizationMode.FUSED_MEGATAG2;
    private int m_singleTagId = -1;
    private double m_lastSingleTagSeenTimestamp = 0.0;

    public Vision(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();

        m_aimController = new PIDController(3.5, 0.8, 0.2);
        m_aimController.enableContinuousInput(-Math.PI, Math.PI);
        m_aimController.setTolerance(Math.toRadians(1.0));
        m_aimController.setIZone(Math.toRadians(10.0));
    }

    // =========================================================================
    // LOCALIZATION MODE API
    // =========================================================================

    /**
     * Switches to single-tag localization mode for final climb approach.
     * MegaTag2 updates are suppressed; only {@code tagId} is used.
     * Call {@link #resumeFusedMode()} to revert.
     *
     * @param tagId  AprilTag ID to use for pose estimation (nearest tower tag)
     */
    public void setLocalizationMode(LocalizationMode mode, int tagId) {
        m_localizationMode = mode;
        m_singleTagId = tagId;
        m_lastSingleTagSeenTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Returns to normal MegaTag2 fused localization.
     * Called when autoclimb completes or aborts.
     */
    public void resumeFusedMode() {
        m_localizationMode = LocalizationMode.FUSED_MEGATAG2;
        m_singleTagId = -1;
    }

    public LocalizationMode getLocalizationMode() {
        return m_localizationMode;
    }

    // =========================================================================
    // EXISTING PUBLIC API
    // =========================================================================

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

        if (!cachedClimbTV) return 0.0;

        double txRadians = Math.toRadians(cachedClimbTX);
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
        return FieldAiming.getDistanceToHub(currentPose);
    }

    /** Checks if the robot is within shooting range. */
    public boolean isInShootingRange() {
        Pose2d currentPose = drivetrain.getState().Pose;
        return FieldAiming.isInScoringRange(currentPose);
    }

    // =========================================================================
    // PERIODIC
    // =========================================================================

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

        double txSum = 0.0;
        int hubTagCount = 0;
        double climbTxSum = 0.0;
        int climbHubTagCount = 0;

        var alliance = DriverStation.getAlliance();
        int[] hubTagIds = (alliance.isPresent() && alliance.get() == Alliance.Red)
            ? VisionConstants.RED_HUB_TAG_IDS
            : VisionConstants.BLUE_HUB_TAG_IDS;

        boolean rejectAllUpdates =
            Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720;

        if (m_localizationMode == LocalizationMode.SINGLE_TAG) {
            updateSingleTagLocalization(currentPose, rejectAllUpdates);
        }

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

            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            SmartDashboard.putBoolean("Vision/" + limelightName + "/TV", tv);
            SmartDashboard.putNumber("Vision/" + limelightName + "/TX", tx);
            SmartDashboard.putNumber("Vision/" + limelightName + "/TagID", tagId);

            if (mt2 != null && mt2.tagCount >= 1) {
                SmartDashboard.putNumber("Vision/" + limelightName + "/TagDist", mt2.avgTagDist);
                SmartDashboard.putNumber("Vision/" + limelightName + "/TagCount", mt2.tagCount);
                SmartDashboard.putNumber("Vision/" + limelightName + "/PoseX", mt2.pose.getX());
                SmartDashboard.putNumber("Vision/" + limelightName + "/PoseY", mt2.pose.getY());

                double poseX = mt2.pose.getX();
                double poseY = mt2.pose.getY();
                if (poseX < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseX > VisionConstants.FIELD_LENGTH_METERS + VisionConstants.FIELD_BORDER_MARGIN
                    || poseY < -VisionConstants.FIELD_BORDER_MARGIN
                    || poseY > VisionConstants.FIELD_WIDTH_METERS + VisionConstants.FIELD_BORDER_MARGIN) {
                    continue;
                }

                boolean isHubTag = false;
                for (int id : hubTagIds) {
                    if (tagId == id) {
                        isHubTag = true;
                        break;
                    }
                }

                if (tv && isHubTag) {
                    txSum += tx;
                    hubTagCount++;

                    if (limelightName.equals(VisionConstants.LIMELIGHT_CLIMB)) {
                        climbTxSum += tx;
                        climbHubTagCount++;
                    }

                    if (mt2.avgTagDist < bestTagDist) {
                        bestTagDist = mt2.avgTagDist;
                        cachedTagDist = mt2.avgTagDist;
                    }
                }

                // Only feed MegaTag2 in FUSED mode
                if (!rejectAllUpdates && m_localizationMode == LocalizationMode.FUSED_MEGATAG2) {
                    double xyStdDev = Math.max(0.5, 0.7 * mt2.avgTagDist / mt2.tagCount);

                    SmartDashboard.putNumber("Vision/" + limelightName + "/StdDev", xyStdDev);
                    SmartDashboard.putBoolean("Vision/" + limelightName + "/PoseAccepted", true);

                    drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
                } else if (m_localizationMode == LocalizationMode.SINGLE_TAG) {
                    SmartDashboard.putBoolean("Vision/" + limelightName + "/PoseAccepted", false);
                    SmartDashboard.putString("Vision/" + limelightName + "/RejectReason",
                        "SINGLE_TAG mode active");
                } else {
                    SmartDashboard.putBoolean("Vision/" + limelightName + "/PoseAccepted", false);
                    SmartDashboard.putString("Vision/" + limelightName + "/RejectReason",
                        "High angular velocity");
                }
            }
        }

        if (hubTagCount > 0) {
            cachedTV = true;
            cachedTX = txSum / hubTagCount;
        }

        if (climbHubTagCount > 0) {
            cachedClimbTV = true;
            cachedClimbTX = climbTxSum / climbHubTagCount;
        }

        SmartDashboard.putNumber("Vision/DistanceToGoal", getDistanceToGoal());
        SmartDashboard.putBoolean("Vision/InShootingRange", isInShootingRange());
        SmartDashboard.putBoolean("Vision/ClimbLimelightTV", cachedClimbTV);
        SmartDashboard.putNumber("Vision/ClimbLimelightTX", cachedClimbTX);
        SmartDashboard.putNumber("Vision/ClimbAimOutput", limelight_aim_proportional());
        SmartDashboard.putString("Vision/LocalizationMode", m_localizationMode.name());

        Pose2d pose = drivetrain.getState().Pose;
        double targetAngle = isInShootingRange()
            ? Math.atan2(getGoalY() - pose.getY(), getGoalX() - pose.getX())
            : (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red ? 0 : Math.PI);
        double currentHeading = pose.getRotation().getRadians();
        double error = targetAngle - currentHeading;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        SmartDashboard.putNumber("Vision/TargetAngleDeg", Math.toDegrees(targetAngle));
        SmartDashboard.putNumber("Vision/CurrentHeadingDeg", Math.toDegrees(currentHeading));
        SmartDashboard.putNumber("Vision/AngleErrorDeg", Math.toDegrees(error));
        SmartDashboard.putBoolean("Vision/PIDAtSetpoint", m_aimController.atSetpoint());
    }

    // =========================================================================
    // SINGLE-TAG LOCALIZATION (called from periodic when in SINGLE_TAG mode)
    // =========================================================================

    /**
     * Checks limelight-right and limelight-left for the target tag ID.
     * If found, feeds a tight-stddev pose estimate to the drivetrain.
     * If not found for SINGLE_TAG_TIMEOUT_S, logs a warning and reverts to FUSED_MEGATAG2.
     */
    private void updateSingleTagLocalization(Pose2d currentPose, boolean rejectHighOmega) {
        if (rejectHighOmega) return;

        boolean tagFound = false;

        String[] nearLimelights = {
            VisionConstants.LIMELIGHT_RIGHT,
            VisionConstants.LIMELIGHT_LEFT
        };

        for (String limelightName : nearLimelights) {
            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            boolean tv  = LimelightHelpers.getTV(limelightName);

            if (!tv || tagId != m_singleTagId) continue;

            // Use single-tag (non-MegaTag2) pose estimate for maximum accuracy
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                currentPose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );

            LimelightHelpers.PoseEstimate singleTagEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

            if (singleTagEstimate == null || singleTagEstimate.tagCount < 1) continue;

            double poseX = singleTagEstimate.pose.getX();
            double poseY = singleTagEstimate.pose.getY();
            if (poseX < -VisionConstants.FIELD_BORDER_MARGIN
                || poseX > VisionConstants.FIELD_LENGTH_METERS + VisionConstants.FIELD_BORDER_MARGIN
                || poseY < -VisionConstants.FIELD_BORDER_MARGIN
                || poseY > VisionConstants.FIELD_WIDTH_METERS + VisionConstants.FIELD_BORDER_MARGIN) {
                continue;
            }

            // Feed with tight stddev — we're very close to a known reference
            drivetrain.addVisionMeasurement(
                singleTagEstimate.pose,
                singleTagEstimate.timestampSeconds,
                VecBuilder.fill(AutoclimbConstants.SINGLE_TAG_XY_STD_DEV,
                                AutoclimbConstants.SINGLE_TAG_XY_STD_DEV,
                                9999999)
            );

            SmartDashboard.putBoolean("Vision/SingleTag/Found", true);
            SmartDashboard.putNumber("Vision/SingleTag/TagId", tagId);
            SmartDashboard.putString("Vision/SingleTag/Limelight", limelightName);

            m_lastSingleTagSeenTimestamp = Timer.getFPGATimestamp();
            tagFound = true;
            break; // Use first limelight that sees the tag
        }

        if (!tagFound) {
            SmartDashboard.putBoolean("Vision/SingleTag/Found", false);

            double timeSinceSeen = Timer.getFPGATimestamp() - m_lastSingleTagSeenTimestamp;
            if (timeSinceSeen > AutoclimbConstants.SINGLE_TAG_TIMEOUT_S) {
                // Tag lost for too long — fall back to fused mode with a warning
                SmartDashboard.putString("Vision/SingleTag/FallbackReason",
                    "Tag " + m_singleTagId + " not seen for " +
                    String.format("%.1f", timeSinceSeen) + "s — reverting to MegaTag2");
                m_localizationMode = LocalizationMode.FUSED_MEGATAG2;
            }
        }
    }
}
