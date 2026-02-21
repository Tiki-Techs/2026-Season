// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CTRE Phoenix 6 imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// PathPlanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// WPILib Math imports
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

// WPILib Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// WPILib SmartDashboard imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Robot Constants imports
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterIntakeConstants;
import frc.robot.Constants.VisionConstants;

// Robot Subsystem imports
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterIntake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

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
    private final double maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND;

    /** Maximum rotational speed in radians per second */
    private final double maxAngularRate = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    // ==================== SLEW RATE LIMITERS ====================
    // These limit acceleration to prevent wheel slip and provide smoother control

    /** X-axis (forward/back) acceleration limiter - 3 m/s per second */
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);

    /** Y-axis (left/right) acceleration limiter - 3 m/s per second */
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);

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

    /** Limelight-assisted drive - no deadband for precise vision control */
    private final SwerveRequest.FieldCentric limelight = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // ==================== CONTROLLERS ====================

    /** Primary driver controller (Xbox) on port 0 */
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // ==================== CONSTRUCTOR ====================

    /**
     * Constructs the RobotContainer.
     * Registers PathPlanner commands, builds auto chooser, and configures bindings.
     */
    public RobotContainer() {
        // Register PathPlanner named commands for autonomous routines
        registerNamedCommands();

        // Build autonomous chooser from PathPlanner paths
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure controller button bindings
        configureBindings();
    }

    // ==================== PATHPLANNER NAMED COMMANDS ====================

    /**
     * Registers all named commands for PathPlanner autonomous routines.
     * These commands can be called by name in PathPlanner paths.
     */
    private void registerNamedCommands() {
        // ----- Shooter Commands -----
        NamedCommands.registerCommand("runPIDShooter",
            m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS).withTimeout(5));
        NamedCommands.registerCommand("stopPIDShooter",
            m_shooter.stopShooter().withTimeout(0.1));
        NamedCommands.registerCommand("runShooter",
            m_shooter.runShooter(ShooterConstants.SHOOTER_DEFAULT_SPEED).withTimeout(0.1));

        // ----- Shooter Intake Commands -----
        NamedCommands.registerCommand("runShooterIntake",
            m_shooterIntake.runShooterIntake(ShooterIntakeConstants.SHOOTER_INTAKE_SPEED).withTimeout(0.1));

        // ----- Index Commands -----
        NamedCommands.registerCommand("runIndex",
            m_index.runIndex(IndexConstants.INDEX_SPEED).withTimeout(5));
        NamedCommands.registerCommand("runReverseIndex",
            m_index.runIndex(-IndexConstants.INDEX_SPEED).withTimeout(5));
        NamedCommands.registerCommand("stopIndex",
            m_index.stopIndex().withTimeout(0.1));

        // ----- Intake Commands -----
        NamedCommands.registerCommand("runIntake",
            m_intake.runIntake(IntakeConstants.INTAKE_SPEED).withTimeout(3));
        NamedCommands.registerCommand("runReverseIntake",
            m_intake.runIntake(-IntakeConstants.INTAKE_SPEED).withTimeout(3));
        NamedCommands.registerCommand("stopIntake",
            m_intake.stopIntake().withTimeout(0.1));

        // ----- Intake Pivot Commands -----
        NamedCommands.registerCommand("raiseArmManual",
            m_intakePivot.raiseArmManual(IntakePivotConstants.PIVOT_SPEED).withTimeout(1));
        NamedCommands.registerCommand("lowerArmManual",
            m_intakePivot.lowerArmManual(IntakePivotConstants.PIVOT_SPEED).withTimeout(1));
        NamedCommands.registerCommand("stopIntakePivot",
            m_intakePivot.stopAll().withTimeout(0.1));
        NamedCommands.registerCommand("changeDeployState",
            m_intakePivot.changeDeployState().withTimeout(0.1));

        // ----- Chained Commands -----
        NamedCommands.registerCommand("PIDShooterAndShooterIntake", PIDShooterAndShooterIntake());

        // ----- Vision Commands -----
        NamedCommands.registerCommand("autoAlign", drivetrain.applyRequest(() ->
            limelight
                .withVelocityX(xLimiter.calculate(-m_Vision.limelight_range_proportional()))
                .withVelocityY(0)
                .withRotationalRate(m_Vision.limelight_aim_proportional())).withTimeout(2));
    }

    // ==================== BUTTON BINDINGS ====================

    /**
     * Configures all controller button bindings.
     * Maps controller inputs to robot commands.
     *
     * CONTROL SCHEME:
     * - Left Stick: Translation (forward/back, left/right)
     * - Right Stick X: Rotation
     * - Right Stick Y: Hood adjustment
     * - Left Trigger: Manual shooter (speed = trigger position)
     * - Right Trigger: Robot-centric drive mode (hold)
     * - Left Bumper: Intake rollers
     * - Right Bumper: PID Shooter (auto-feeds when at speed)
     * - A Button: Limelight auto-aim drive
     * - B Button: Shooter intake
     * - X Button: Index toggle
     * - Y Button: Override mode (hold to reverse mechanisms)
     * - D-Pad Up: Raise intake arm
     * - D-Pad Down: Lower intake arm
     * - Start Button: Reset gyro heading
     */
    private void configureBindings() {
        // ========== DRIVETRAIN ==========
        configureDrivetrainBindings();

        // ========== SHOOTER ==========
        configureShooterBindings();

        // ========== INTAKE ==========
        configureIntakeBindings();

        // ========== DEFAULT COMMANDS ==========
        configureDefaultCommands();

        // ========== LIMELIGHT POWER MANAGEMENT ==========
        configureLimelightPowerManagement();
    }

    /**
     * Configures drivetrain-related button bindings.
     */
    private void configureDrivetrainBindings() {
        // Default Command: Field-centric drive, switches to robot-centric when right trigger held
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                if (m_driverController.getRightTriggerAxis() > 0.1) {
                    // Robot-centric drive while right trigger held
                    return rDrive
                        .withVelocityX(m_driverController.getLeftY() * maxSpeed)
                        .withVelocityY(m_driverController.getLeftX() * maxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * maxAngularRate);
                } else {
                    // Field-centric drive (default)
                    return drive
                        .withVelocityX(m_driverController.getLeftY() * maxSpeed)
                        .withVelocityY(m_driverController.getLeftX() * maxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * maxAngularRate);
                }
            })
        );

        // A Button: Limelight auto-aim drive - automatically aims and approaches target
        m_driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                limelight
                    .withVelocityX(-xLimiter.calculate(-m_Vision.limelight_range_proportional()))
                    .withVelocityY(-yLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed))
                    .withRotationalRate(m_Vision.limelight_aim_proportional())
            )
        );

        // Start Button: Reset gyro heading to current orientation
        m_driverController.start().onTrue(new InstantCommand(() ->
            drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                new Rotation2d()
            ))
        ));
    }

    /**
     * Configures shooter-related button bindings.
     */
    private void configureShooterBindings() {
        // Y Button: Override mode - hold to reverse direction of mechanisms
        m_driverController.y()
            .onTrue(new InstantCommand(() -> Constants.overrideEnabled = true))
            .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));

        // Left Trigger: Manual shooter - speed proportional to trigger position
        // Normal: forward | Override: reverse
        m_driverController.leftTrigger().whileTrue(
            new ConditionalCommand(
                m_shooter.runReverseShooter(m_driverController),
                m_shooter.runShooter(m_driverController),
                () -> Constants.overrideEnabled
            )
        );

        // Right Bumper: PID shooter with automatic index and shooter intake
        // Normal: spins up shooter, then feeds when at speed
        // Override: reverses all (shooter, index, shooterIntake)
        m_driverController.rightBumper().whileTrue(
            new ConditionalCommand(
                // Override: reverse all mechanisms
                new ParallelCommandGroup(
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    m_index.runIndex(-IndexConstants.INDEX_SPEED),
                    m_shooterIntake.runShooterIntake(-ShooterIntakeConstants.SHOOTER_INTAKE_SPEED)
                ),
                // Normal: spin up shooter, then run all when at speed
                new SequentialCommandGroup(
                    m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS)
                        .until(() -> m_shooter.isAtTargetSpeed(ShooterConstants.SHOOTER_TARGET_RPS, 5.0)),
                    new ParallelCommandGroup(
                        m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                        m_index.runIndex(IndexConstants.INDEX_SPEED),
                        m_shooterIntake.runShooterIntake(ShooterIntakeConstants.SHOOTER_INTAKE_SPEED)
                    )
                ),
                () -> Constants.overrideEnabled
            )
        );

        // B Button: Shooter intake only
        // Normal: forward | Override: reverse
        m_driverController.b().whileTrue(
            new ConditionalCommand(
                m_shooterIntake.runShooterIntake(-ShooterIntakeConstants.SHOOTER_INTAKE_SPEED),
                m_shooterIntake.runShooterIntake(ShooterIntakeConstants.SHOOTER_INTAKE_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // X Button: Toggle index belt
        // Normal: feed forward | Override: reverse
        m_driverController.x().toggleOnTrue(
            new ConditionalCommand(
                m_index.runIndex(-1),
                m_index.runIndex(1),
                () -> Constants.overrideEnabled
            )
        );
    }

    /**
     * Configures intake-related button bindings.
     */
    private void configureIntakeBindings() {
        // D-Pad Up: Raise intake arm
        m_driverController.povUp().whileTrue(
            new ConditionalCommand(
                m_intakePivot.raiseArmManual(IntakePivotConstants.PIVOT_SPEED),
                m_intakePivot.raiseArmManual(IntakePivotConstants.PIVOT_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // D-Pad Down: Lower intake arm
        m_driverController.povDown().whileTrue(
            new ConditionalCommand(
                m_intakePivot.lowerArmManual(IntakePivotConstants.PIVOT_SPEED),
                m_intakePivot.lowerArmManual(IntakePivotConstants.PIVOT_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // Left Bumper: Run intake rollers
        // Normal: intake | Override: eject
        m_driverController.leftBumper().whileTrue(
            new ConditionalCommand(
                m_intake.runIntake(1),
                m_intake.runIntake(-1),
                () -> Constants.overrideEnabled
            )
        );
    }

    /**
     * Configures default commands for all subsystems.
     * These run when no other command is using the subsystem.
     */
    private void configureDefaultCommands() {
        m_shooter.setDefaultCommand(m_shooter.stopAll());
        m_shooterIntake.setDefaultCommand(m_shooterIntake.stopAll());
        m_intake.setDefaultCommand(m_intake.stopAll());
        m_intakePivot.setDefaultCommand(m_intakePivot.stopAll());
        m_index.setDefaultCommand(m_index.stopAll());

        // Hood: controlled by right stick Y axis
        m_hood.setDefaultCommand(
            m_hood.runHood(() -> MathUtil.applyDeadband(m_driverController.getRightY(), 0.15))
        );
    }

    /**
     * Configures Limelight power management based on robot mode.
     * Reduces processing when disabled to save power.
     */
    private void configureLimelightPowerManagement() {
        // Throttle Limelight when robot is disabled
        RobotModeTriggers.disabled()
            .onTrue(Commands.runOnce(() ->
                LimelightHelpers.setLimelightNTDouble(VisionConstants.LIMELIGHT_NAME, "throttle", 150)
            ));

        // Full speed when robot is enabled (teleop or autonomous)
        RobotModeTriggers.teleop().or(RobotModeTriggers.autonomous())
            .onTrue(Commands.runOnce(() ->
                LimelightHelpers.setLimelightNTDouble(VisionConstants.LIMELIGHT_NAME, "throttle", 0)
            ));
    }

    // ==================== AUTONOMOUS ====================

    /**
     * Returns the autonomous command selected from SmartDashboard.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // ==================== CHAINED COMMANDS ====================

    /**
     * Creates a command that runs the shooter at target speed,
     * then feeds with index and shooterIntake when at speed.
     *
     * @return The chained shooting command
     */
    public Command PIDShooterAndShooterIntake() {
        return new SequentialCommandGroup(
            // Spin up shooter until at target speed
            m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS)
                .until(() -> m_shooter.isAtTargetSpeed(ShooterConstants.SHOOTER_TARGET_RPS, 5.0)),
            // Once at speed, run shooter, index, and shooterIntake together
            new ParallelCommandGroup(
                m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                m_index.runIndex(IndexConstants.INDEX_SPEED),
                m_shooterIntake.runShooterIntake(ShooterIntakeConstants.SHOOTER_INTAKE_SPEED)
            )
        );
    }
}
