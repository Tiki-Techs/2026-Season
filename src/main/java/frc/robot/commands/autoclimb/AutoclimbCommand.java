package frc.robot.commands.autoclimb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LocalizationMode;

/**
 * Autonomous climb sequence command.
 *
 * State machine: IDLE → PATHFINDING → FINAL_APPROACH → ALIGNED_HOLD → ENGAGING → LIFTING → CLIMBED
 *
 * Abort: interrupting the command (release button) triggers finallyDo in RobotContainer,
 * which calls emergencyRetract() and resumeFusedMode().
 *
 * PREREQUISITE: m_climb.calibrateClimb() must have been run this power cycle
 * before triggering this command.
 */
public class AutoclimbCommand extends Command {

    // -------------------------------------------------------------------------
    // State definitions
    // -------------------------------------------------------------------------

    private enum State {
        IDLE, PATHFINDING, FINAL_APPROACH, ALIGNED_HOLD,
        ENGAGING, LIFTING, CLIMBED, ABORTED
    }

    private enum LiftPhase { RAISING, LIFTING_PHASE }

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    private final SwerveSubsystem drivetrain;
    private final Climb climb;
    private final Vision vision;

    // -------------------------------------------------------------------------
    // State tracking
    // -------------------------------------------------------------------------

    private State currentState = State.IDLE;
    private LiftPhase liftPhase = LiftPhase.RAISING;
    private EngagementTarget target;

    // Sub-commands managed with manual lifecycle (inline delegation pattern —
    // avoids scheduler requirement conflicts while AutoclimbCommand holds drivetrain).
    private Command pathfindCommand;
    private Command raiseCommand;
    private Command liftSubCommand;

    // -------------------------------------------------------------------------
    // Swerve requests
    // -------------------------------------------------------------------------

    // FINAL_APPROACH + ALIGNED_HOLD: hold heading while translating to capture pose
    private final SwerveRequest.FieldCentricFacingAngle holdHeading =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ENGAGING: drive straight backward in robot frame
    private final SwerveRequest.RobotCentric engageRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // CLIMBED / ABORTED: lock wheels
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // -------------------------------------------------------------------------
    // Translation PID for FINAL_APPROACH
    // -------------------------------------------------------------------------

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;

    // -------------------------------------------------------------------------
    // Timers
    // -------------------------------------------------------------------------

    private final Timer stateTimer       = new Timer();
    private final Timer alignedTimer     = new Timer(); // debounce within FINAL_APPROACH
    private final Timer currentSpikeTimer = new Timer(); // ENGAGING current spike duration

    // -------------------------------------------------------------------------
    // ENGAGING tracking
    // -------------------------------------------------------------------------

    private Pose2d engageStartPose;
    private boolean exitReasonLogged = false;

    // =========================================================================
    // Constructor
    // =========================================================================

    public AutoclimbCommand(SwerveSubsystem drivetrain, Climb climb, Vision vision) {
        this.drivetrain = drivetrain;
        this.climb = climb;
        this.vision = vision;

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(
            AutoclimbConstants.APPROACH_MAX_SPEED_MPS,
            AutoclimbConstants.APPROACH_MAX_ACCEL_MPSS
        );
        xController = new ProfiledPIDController(
            AutoclimbConstants.TRANSLATION_KP, 0, 0, translationConstraints);
        yController = new ProfiledPIDController(
            AutoclimbConstants.TRANSLATION_KP, 0, 0, translationConstraints);
        xController.setTolerance(AutoclimbConstants.X_TOLERANCE_M);
        yController.setTolerance(AutoclimbConstants.Y_TOLERANCE_M);

        holdHeading.HeadingController.setPID(10.0, 0.0, 0.1);
        holdHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, climb);
    }

    // =========================================================================
    // Command lifecycle
    // =========================================================================

    @Override
    public void initialize() {
        target = EngagementTarget.getDefault(
            DriverStation.getAlliance().orElse(null));
        currentState = State.IDLE;
        exitReasonLogged = false;
        cleanupSubCommands(true);
        transitionTo(State.PATHFINDING);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;

        publishTelemetry(pose);

        switch (currentState) {

            case PATHFINDING: {
                pathfindCommand.execute();
                boolean timedOut = stateTimer.hasElapsed(AutoclimbConstants.PATHFINDING_TIMEOUT_S);
                if (pathfindCommand.isFinished() || timedOut) {
                    pathfindCommand.end(timedOut);
                    pathfindCommand = null;
                    transitionTo(State.FINAL_APPROACH);
                }
                break;
            }

            case FINAL_APPROACH: {
                Pose2d capturePose = target.getCapturePose();

                double vx = xController.calculate(pose.getX());
                double vy = yController.calculate(pose.getY());
                drivetrain.setControl(
                    holdHeading
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(target.targetHeading)
                );

                boolean xOk = Math.abs(capturePose.getX() - pose.getX()) < AutoclimbConstants.X_TOLERANCE_M;
                boolean yOk = Math.abs(capturePose.getY() - pose.getY()) < AutoclimbConstants.Y_TOLERANCE_M;
                double headingErrRad = Math.abs(
                    target.targetHeading.minus(pose.getRotation()).getRadians());
                boolean hOk = headingErrRad < AutoclimbConstants.HEADING_TOLERANCE_RAD;

                if (xOk && yOk && hOk) {
                    alignedTimer.start();
                } else {
                    alignedTimer.reset();
                    alignedTimer.stop();
                }

                if (alignedTimer.hasElapsed(AutoclimbConstants.TOLERANCE_DEBOUNCE_S)) {
                    transitionTo(State.ALIGNED_HOLD);
                } else if (stateTimer.hasElapsed(AutoclimbConstants.FINAL_APPROACH_TIMEOUT_S)) {
                    // Proceed anyway — best-effort alignment
                    transitionTo(State.ALIGNED_HOLD);
                }
                break;
            }

            case ALIGNED_HOLD: {
                drivetrain.setControl(
                    holdHeading
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withTargetDirection(target.targetHeading)
                );
                if (stateTimer.hasElapsed(AutoclimbConstants.ALIGNED_HOLD_TIMEOUT_S)) {
                    transitionTo(State.ENGAGING);
                }
                break;
            }

            case ENGAGING: {
                // Slide parallel to driver station wall (±Y) so pipe threads through N opening
                drivetrain.setControl(
                    engageRequest
                        .withVelocityX(0)
                        .withVelocityY(-target.approachYSign * AutoclimbConstants.ENGAGE_VELOCITY_MPS)
                        .withRotationalRate(0)
                );

                // Measure Y-axis displacement only (we're sliding in Y)
                double distTraveled = Math.abs(pose.getY() - engageStartPose.getY());
                double avgCurrent = drivetrain.getAverageDriveStatorCurrent();

                SmartDashboard.putNumber("Autoclimb/EngageDistM", distTraveled);
                SmartDashboard.putNumber("Autoclimb/AvgStatorA", avgCurrent);

                if (avgCurrent > AutoclimbConstants.ENGAGE_CURRENT_SPIKE_A) {
                    currentSpikeTimer.start();
                } else {
                    currentSpikeTimer.reset();
                    currentSpikeTimer.stop();
                }

                boolean distDone    = distTraveled >= AutoclimbConstants.ENGAGE_DISTANCE_M;
                boolean currentDone = currentSpikeTimer.hasElapsed(
                    AutoclimbConstants.ENGAGE_CURRENT_SPIKE_DURATION_S);
                boolean timedOut    = stateTimer.hasElapsed(AutoclimbConstants.ENGAGE_TIMEOUT_S);

                if (!exitReasonLogged && (distDone || currentDone || timedOut)) {
                    String reason = distDone ? "distance" : (currentDone ? "current" : "timeout");
                    SmartDashboard.putString("Autoclimb/ExitReason", reason);
                    exitReasonLogged = true;

                    if (timedOut && !distDone && !currentDone) {
                        transitionTo(State.ABORTED);
                    } else {
                        transitionTo(State.LIFTING);
                    }
                }
                break;
            }

            case LIFTING: {
                switch (liftPhase) {
                    case RAISING:
                        raiseCommand.execute();
                        // Switch to pull-down once arm reaches upper limit switch
                        if (climb.isUpperLimitPressed()) {
                            raiseCommand.end(false);
                            raiseCommand = null;
                            liftSubCommand = climb.runClimbDown();
                            liftSubCommand.initialize();
                            liftPhase = LiftPhase.LIFTING_PHASE;
                        }
                        break;

                    case LIFTING_PHASE:
                        liftSubCommand.execute();
                        break;
                }
                if (stateTimer.hasElapsed(AutoclimbConstants.LIFTING_TIMEOUT_S)) {
                    transitionTo(State.CLIMBED);
                }
                break;
            }

            case CLIMBED:
            case ABORTED:
                drivetrain.setControl(brake);
                break;

            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentState == State.CLIMBED || currentState == State.ABORTED;
    }

    @Override
    public void end(boolean interrupted) {
        cleanupSubCommands(interrupted);
        drivetrain.setControl(brake);
        SmartDashboard.putString("Autoclimb/State",
            interrupted ? "ABORTED(interrupted)" : currentState.name());
        // emergencyRetract() and resumeFusedMode() are handled by finallyDo in RobotContainer
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    private void transitionTo(State next) {
        // Exit cleanup for current state
        switch (currentState) {
            case PATHFINDING:
                if (pathfindCommand != null) {
                    pathfindCommand.end(true);
                    pathfindCommand = null;
                }
                break;
            case LIFTING:
                if (raiseCommand != null) { raiseCommand.end(true); raiseCommand = null; }
                if (liftSubCommand != null) { liftSubCommand.end(true); liftSubCommand = null; }
                break;
            default:
                break;
        }

        currentState = next;
        stateTimer.reset();
        stateTimer.start();
        SmartDashboard.putString("Autoclimb/State", next.name());

        // Entry actions for new state
        switch (next) {
            case PATHFINDING:
                pathfindCommand = AutoBuilder.pathfindToPose(
                    target.getStandoffPose(),
                    AutoclimbConstants.PATH_CONSTRAINTS
                );
                pathfindCommand.initialize();
                break;

            case FINAL_APPROACH:
                Pose2d capturePose = target.getCapturePose();
                xController.reset(drivetrain.getState().Pose.getX());
                yController.reset(drivetrain.getState().Pose.getY());
                xController.setGoal(capturePose.getX());
                yController.setGoal(capturePose.getY());
                vision.setLocalizationMode(LocalizationMode.SINGLE_TAG, target.nearestTagId);
                alignedTimer.reset();
                alignedTimer.stop();
                break;

            case ALIGNED_HOLD:
                // stateTimer is now the hold timer (reset in transitionTo above)
                break;

            case ENGAGING:
                engageStartPose = drivetrain.getState().Pose;
                currentSpikeTimer.reset();
                currentSpikeTimer.stop();
                exitReasonLogged = false;
                break;

            case LIFTING:
                // Use limit-switch-based raise — no encoder calibration required
                raiseCommand = climb.runClimbUp();
                raiseCommand.initialize();
                liftPhase = LiftPhase.RAISING;
                break;

            case CLIMBED:
            case ABORTED:
            default:
                break;
        }
    }

    private void cleanupSubCommands(boolean interrupted) {
        if (pathfindCommand != null) {
            pathfindCommand.end(interrupted);
            pathfindCommand = null;
        }
        if (raiseCommand != null) {
            raiseCommand.end(interrupted);
            raiseCommand = null;
        }
        if (liftSubCommand != null) {
            liftSubCommand.end(interrupted);
            liftSubCommand = null;
        }
    }

    private void publishTelemetry(Pose2d pose) {
        SmartDashboard.putString("Autoclimb/State", currentState.name());
        if (target == null) return;

        Pose2d capturePose = target.getCapturePose();
        SmartDashboard.putString("Autoclimb/Target",    target.getDisplayName());
        SmartDashboard.putNumber("Autoclimb/TargetX",   capturePose.getX());
        SmartDashboard.putNumber("Autoclimb/TargetY",   capturePose.getY());
        SmartDashboard.putNumber("Autoclimb/PoseErrorX",
            capturePose.getX() - pose.getX());
        SmartDashboard.putNumber("Autoclimb/PoseErrorY",
            capturePose.getY() - pose.getY());
        SmartDashboard.putNumber("Autoclimb/HeadingErrorDeg",
            target.targetHeading.minus(pose.getRotation()).getDegrees());
    }
}
