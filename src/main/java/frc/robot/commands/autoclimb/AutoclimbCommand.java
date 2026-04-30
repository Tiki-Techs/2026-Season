package frc.robot.commands.autoclimb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

/**
 * Autonomous climb sequence command.
 *
 * State machine:
 *   PATHFINDING -> RAISE_CLIMB -> DRIVE_BACKWARD -> LOWER_AND_LIFT -> CLIMBED
 *
 * Sequence:
 *   1. PathPlanner navigates to the target pose beside the rung (robot back facing structure).
 *   2. Arm raises until upper limit switch fires or timeout. Drivetrain locked.
 *   3. Robot drives robot-relative backward (-X) for DRIVE_BACKWARD_TIMEOUT_S seconds.
 *   4. Arm lowers slowly until lower limit switch fires or timeout. Robot lifts.
 *   5. Drivetrain brakes lock. Motor holds in brake mode.
 *
 * The climb motor is driven via direct method calls (raiseMotor / lowerMotorSlow)
 * rather than scheduled sub-commands. This prevents the stopAll() default command
 * from fighting the climb motor while AutoclimbCommand is running.
 *
 * Abort: interrupting triggers finallyDo in RobotContainer.
 */
public class AutoclimbCommand extends Command {

    private enum State {
        PATHFINDING, RAISE_CLIMB, DRIVE_BACKWARD, LOWER_AND_LIFT, CLIMBED, ABORTED
    }

    private final SwerveSubsystem drivetrain;
    private final Climb climb;
    private final Vision vision;

    private State currentState;
    private EngagementTarget target;

    private Command pathfindCommand;

    // DRIVE_BACKWARD: robot-centric -X. At 90 deg (Blue) or -90 deg (Red) heading,
    // robot -X points toward the rung structure.
    private final SwerveRequest.RobotCentric driveBackward =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(-AutoclimbConstants.BACKWARD_DRIVE_SPEED_MPS)
            .withVelocityY(0)
            .withRotationalRate(0);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Timer stateTimer = new Timer();

    public AutoclimbCommand(SwerveSubsystem drivetrain, Climb climb, Vision vision) {
        this.drivetrain = drivetrain;
        this.climb      = climb;
        this.vision     = vision;
        // Only require drivetrain here. Climb motor is driven via direct calls
        // so the stopAll() default command is naturally interrupted when this
        // command starts (AutoclimbCommand holds the climb requirement too,
        // which cancels stopAll). We still need the requirement so stopAll
        // doesn't restart mid-sequence.
        addRequirements(drivetrain, climb);
    }

    @Override
    public void initialize() {
        target = EngagementTarget.getDefault(
            DriverStation.getAlliance().orElse(null));
        if (pathfindCommand != null) { pathfindCommand.end(true); pathfindCommand = null; }
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
                    transitionTo(State.RAISE_CLIMB);
                }
                break;
            }

            case RAISE_CLIMB: {
                drivetrain.setControl(brake);
                climb.raiseMotor();

                if (climb.isUpperLimitPressed()
                        || stateTimer.hasElapsed(AutoclimbConstants.RAISE_CLIMB_TIMEOUT_S)) {
                    transitionTo(State.DRIVE_BACKWARD);
                }
                break;
            }

            case DRIVE_BACKWARD: {
                drivetrain.setControl(driveBackward);

                if (stateTimer.hasElapsed(AutoclimbConstants.DRIVE_BACKWARD_TIMEOUT_S)) {
                    transitionTo(State.LOWER_AND_LIFT);
                }
                break;
            }

            case LOWER_AND_LIFT: {
                drivetrain.setControl(brake);
                climb.lowerMotorSlow();

                if (climb.isLowerLimitPressed()
                        || stateTimer.hasElapsed(AutoclimbConstants.LOWER_LIFT_TIMEOUT_S)) {
                    transitionTo(State.CLIMBED);
                }
                break;
            }

            case CLIMBED:
            case ABORTED:
                drivetrain.setControl(brake);
                // Motor is already stopped — brake mode holds the robot hung.
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
        if (pathfindCommand != null) { pathfindCommand.end(interrupted); pathfindCommand = null; }
        drivetrain.setControl(brake);
        SmartDashboard.putString("Autoclimb/State",
            interrupted ? "ABORTED(interrupted)" : currentState.name());
    }

    private void transitionTo(State next) {
        if (currentState == State.PATHFINDING && pathfindCommand != null) {
            pathfindCommand.end(true);
            pathfindCommand = null;
        }

        currentState = next;
        stateTimer.reset();
        stateTimer.start();
        SmartDashboard.putString("Autoclimb/State", next.name());

        switch (next) {
            case PATHFINDING:
                pathfindCommand = AutoBuilder.pathfindToPose(
                    target.getTargetPose(),
                    AutoclimbConstants.PATH_CONSTRAINTS
                );
                pathfindCommand.initialize();
                break;

            case CLIMBED:
            case ABORTED:
                vision.resumeFusedMode();
                break;

            default:
                break;
        }
    }

    private void publishTelemetry(Pose2d pose) {
        SmartDashboard.putString("Autoclimb/State",        currentState.name());
        SmartDashboard.putString("Autoclimb/Target",       target != null ? target.getDisplayName() : "none");
        SmartDashboard.putNumber("Autoclimb/RobotX",       pose.getX());
        SmartDashboard.putNumber("Autoclimb/RobotY",       pose.getY());
        SmartDashboard.putNumber("Autoclimb/HeadingDeg",   pose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Autoclimb/ArmAtTop",    climb.isUpperLimitPressed());
        SmartDashboard.putBoolean("Autoclimb/ArmAtBottom", climb.isLowerLimitPressed());
        SmartDashboard.putNumber("Autoclimb/ClimbCurrent", climb.getStatorCurrent());
        SmartDashboard.putNumber("Autoclimb/StateTimerS",  stateTimer.get());
        if (target != null) {
            Pose2d tp = target.getTargetPose();
            SmartDashboard.putNumber("Autoclimb/TargetX",  tp.getX());
            SmartDashboard.putNumber("Autoclimb/TargetY",  tp.getY());
        }
    }
}
