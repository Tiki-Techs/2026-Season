// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.commands.SlowDriveTrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotContainer is the central hub for robot configuration.
 * This class instantiates all subsystems, configures button bindings,
 * and sets up autonomous command selection.
 *
 * In command-based programming, this is where the robot's structure is defined
 * rather than in the Robot periodic methods.
 */
public class RobotContainer {

    // ==================== SUBSYSTEMS ====================

    /** Swerve drivetrain - handles all drive motor control and odometry */
    private final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    /** Vision processing - handles Limelight data and pose estimation */
    private final Vision m_Vision = new Vision(drivetrain);

    /** Slow drive mode utility - limits max speed for testing/tuning */
    private final SlowDriveTrain m_SlowDriveTrain = new SlowDriveTrain();

    /** Shooter subsystem - flywheel for launching game pieces */
    private final Shooter m_shooter = new Shooter();

    /** ShooterIntake subsystem - controls intake roller for shooter */
    private final ShooterIntake m_shooterIntake = new ShooterIntake();

    /** Index subsystem - belt feeder between intake and shooter */
    private final Index m_index = new Index();

    /** Intake subsystem - rollers for collecting game pieces */
    private final Intake m_intake = new Intake();

    /** Intake pivot - raises/lowers the intake arm */
    private final IntakePivot m_intakePivot = new IntakePivot();

    /** Hood subsystem - adjusts shooter launch angle */
    private final Hood m_hood = new Hood();

    // ==================== AUTONOMOUS ====================

    /** Dropdown selector for autonomous routines on SmartDashboard */
    private final SendableChooser<Command> autoChooser;

    // ==================== DRIVE PARAMETERS ====================

    /** Maximum translational speed in meters per second */
    private final double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

    /** Maximum rotational speed in radians per second */
    private final double maxAngularRate = DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // ==================== SLEW RATE LIMITERS ====================
    // These limit acceleration to prevent wheel slip and provide smoother control

    /** X-axis (forward/back) acceleration limiter - 3 m/s per second */
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);

    /** Y-axis (left/right) acceleration limiter - 3 m/s per second */
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

    /** Rotation acceleration limiter - 3 rad/s per second */
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    // ==================== SWERVE REQUESTS ====================
    // Pre-configured drive modes for different situations

    /** Field-centric drive - forward is always field forward, 10% deadband */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /** Robot-centric drive - forward is robot forward, 10% deadband */
    private final SwerveRequest.RobotCentric rDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /** Brake mode - locks wheels in X pattern to resist pushing */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    /** Point wheels - aims all wheels at a specified angle */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /** Limelight-assisted drive - no deadband for precise vision control */
    private final SwerveRequest.FieldCentric limelight = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ==================== CONTROLLERS ====================

    /** Primary driver controller (Xbox) on port 0 */
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * Constructs the RobotContainer.
     * Registers PathPlanner commands, builds auto chooser, and configures bindings.
     */
    public RobotContainer() {
        // ========== PATHPLANNER NAMED COMMANDS ==========
        // These commands can be called by name in PathPlanner autonomous routines

        // Shooter commands
        NamedCommands.registerCommand("runPIDShooter", m_shooter.runPIDShooter(ShooterConstants.shooterTargetRPS).withTimeout(0.1));
        NamedCommands.registerCommand("stopPIDShooter", m_shooter.stopShooter().withTimeout(0.1));
        NamedCommands.registerCommand("runShooter", m_shooter.runShooter(ShooterConstants.shooterDefaultSpeed).withTimeout(0.1));

        NamedCommands.registerCommand("runShooterIntake", m_shooterIntake.runShooterIntake(ShooterIntakeConstants.shooterIntakeSpeed).withTimeout(0.1));

        // Index commands
        NamedCommands.registerCommand("runIndex", m_index.runIndex(IndexConstants.indexSpeed).withTimeout(0.1)); // test timout code/ 
        NamedCommands.registerCommand("runReverseIndex", m_index.runIndex(-IndexConstants.indexSpeed).withTimeout(5));
        NamedCommands.registerCommand("stopIndex", m_index.stopIndex().withTimeout(0.1));

        // Intake commands
        NamedCommands.registerCommand("runIntake", m_intake.runIntake(IntakeConstants.intakeSpeed).withTimeout(0.1));
        NamedCommands.registerCommand("runReverseIntake", m_intake.runIntake(-IntakeConstants.intakeSpeed).withTimeout(5));
        NamedCommands.registerCommand("stopIntake", m_intake.stopIntake().withTimeout(0.1));

        // Intake Pivot commands
        NamedCommands.registerCommand("raiseArmManual", m_intakePivot.raiseArmManual(IntakePivotConstants.pivotSpeed).withTimeout(1));
        NamedCommands.registerCommand("lowerArmManual", m_intakePivot.lowerArmManual(IntakePivotConstants.pivotSpeed).withTimeout(1));
        NamedCommands.registerCommand("stopIntakePivot", m_intakePivot.stopAll().withTimeout(0.1));
        NamedCommands.registerCommand("changeDeployState", m_intakePivot.changeDeployState().withTimeout(0.1));


        NamedCommands.registerCommand("autoAlign", drivetrain.applyRequest(() ->
                limelight
                    .withVelocityX(xLimiter.calculate(-m_Vision.limelight_range_proportional()))
                    .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed))
                    .withRotationalRate(m_Vision.limelight_aim_proportional())).withTimeout(2));


        // Build autonomous chooser from PathPlanner paths
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure controller button bindings
        configureBindings();
    }

    /**
     * Configures all controller button bindings.
     * Maps controller inputs to robot commands.
     *
     * CONTROL SCHEME:
     * - Left Stick: Translation (forward/back, left/right)
     * - Right Stick X: Rotation
     * - Left Trigger: Brake mode
     * - Right Trigger: Manual shooter (speed = trigger position)
     * - Left Bumper: Intake
     * - Right Bumper: PID Shooter
     * - A: Limelight auto-aim drive
     * - B: Hood adjustment
     * - X: Index toggle
     * - Y: Override mode (hold)
     * - D-Pad Up/Down: Intake pivot
     * - Start: Reset gyro heading
     */
    private void configureBindings() {

        // ========== DEFAULT DRIVE COMMAND ==========
        // Field-centric driving with joystick input
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(m_driverController.getLeftY() * maxSpeed)
                    .withVelocityY(m_driverController.getLeftX() * maxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * maxAngularRate)
            )
        );

        // ========== DRIVE CONTROLS ==========

        // Left Trigger - Brake: locks wheels in X pattern
        m_driverController.leftTrigger().whileTrue(drivetrain.applyRequest(() -> brake));

        // A Button - Limelight Assisted Drive: auto-aims and approaches target
        m_driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                limelight
                    .withVelocityX(-xLimiter.calculate(-m_Vision.limelight_range_proportional()))
                    .withVelocityY(-yLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed))
                    .withRotationalRate(m_Vision.limelight_aim_proportional())
            )
        );

        // Start Button - Reset Gyro: resets field-centric forward direction
        m_driverController.start().onTrue(new InstantCommand(() ->
            drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                new Rotation2d()
            ))
        ));

        // ========== OVERRIDE MODE ==========
        // Hold Y to enable override mode - reverses direction of most mechanisms

        m_driverController.y()
            .onTrue(new InstantCommand(() -> Constants.overrideEnabled = true))
            .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));

        // ========== INTAKE PIVOT ==========

        // D-Pad Up - Raise intake arm
        // Normal: auto-raise to limit switch | Override: manual raise
        m_driverController.povUp().whileTrue(
            new ConditionalCommand(
                m_intakePivot.raiseArmManual(IntakePivotConstants.pivotSpeed),
                m_intakePivot.raiseArmManual(IntakePivotConstants.pivotSpeed),

                // m_intakePivot.raiseArmAuto(),
                () -> Constants.overrideEnabled
            )
        );

        // D-Pad Down - Lower intake arm
        // Normal: auto-lower to limit switch | Override: manual lower
        m_driverController.povDown().whileTrue(
            new ConditionalCommand(
                m_intakePivot.lowerArmManual(IntakePivotConstants.pivotSpeed),
                m_intakePivot.lowerArmManual(IntakePivotConstants.pivotSpeed),

                // m_intakePivot.lowerArmAuto(),
                () -> Constants.overrideEnabled
            )
        );

        // ========== INTAKE ==========

        // Left Bumper - Run intake rollers
        // Normal: intake (negative) | Override: eject (positive)
        m_driverController.leftBumper().whileTrue(
            new ConditionalCommand(
                m_intake.runIntake(1),
                m_intake.runIntake(-1),
                () -> Constants.overrideEnabled
            )
        );

        // ========== INDEX ==========

        // X Button - Toggle index belt
        // Normal: feed forward | Override: reverse
        m_driverController.x().toggleOnTrue(
            new ConditionalCommand(
                m_index.runIndex(-1),
                m_index.runIndex(1),
                () -> Constants.overrideEnabled
            )
        );

        // ========== SHOOTER ==========

        // Right Bumper - PID controlled shooter
        // Normal: 100 RPS | Override: -100 RPS (reverse)
        m_driverController.rightBumper().whileTrue(
            new ConditionalCommand(
                m_shooter.runPIDShooter(-ShooterConstants.shooterTargetRPS),

                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(2.0),
                        m_shooter.runPIDShooter(ShooterConstants.shooterTargetRPS)
                    ),
                    new ParallelCommandGroup(
                            m_shooter.runPIDShooter(ShooterConstants.shooterTargetRPS),
                            m_shooterIntake.runShooterIntake(ShooterIntakeConstants.shooterIntakeSpeed)
                    )
                ),
                // override condition
                () -> Constants.overrideEnabled
            )
        );

        // Right Trigger - Manual shooter (speed = trigger position)
        // Normal: forward | Override: reverse
        m_driverController.rightTrigger().whileTrue(
            new ConditionalCommand(
                m_shooter.runReverseShooter(m_driverController),
                m_shooter.runShooter(m_driverController),
                () -> Constants.overrideEnabled
            )
        );

        // ========== HOOD ==========

        // B Button - Adjust hood angle
        // Normal: down (-0.03) | Override: up (+0.03)
        m_driverController.b().whileTrue(
            new ConditionalCommand(
                m_hood.runHood(0.03),
                m_hood.runHood(-0.03),
                () -> Constants.overrideEnabled
            )
        );

        // ========== DEFAULT COMMANDS ==========
        // These run when no other command is using the subsystem

        m_shooter.setDefaultCommand(m_shooter.stopAll());
        m_shooterIntake.setDefaultCommand(m_shooterIntake.stopAll());
        m_intake.setDefaultCommand(m_intake.stopAll());
        m_intakePivot.setDefaultCommand(m_intakePivot.stopAll());
        m_index.setDefaultCommand(m_index.stopAll());
        m_hood.setDefaultCommand(m_hood.stopAll());

        // ========== LIMELIGHT POWER MANAGEMENT ==========
        // Reduce Limelight processing when disabled to save power

        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce(() ->
                LimelightHelpers.setLimelightNTDouble("limelight", "throttle", 150)
            ));

        RobotModeTriggers.teleop().or(RobotModeTriggers.autonomous())
            .onTrue(Commands.runOnce(() ->
                LimelightHelpers.setLimelightNTDouble("limelight", "throttle", 0)
            ));
    }

    /**
     * Returns the autonomous command selected from SmartDashboard.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
