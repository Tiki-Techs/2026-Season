// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

/** Central hub for robot configuration, subsystem instantiation, and button bindings. */
public class RobotContainer {

    // Subsystems
    private final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();
    private final Vision m_vision = new Vision(drivetrain);
    private final Shooter m_shooter = new Shooter();
    private final Feeder m_feeder = new Feeder();
    private final Index m_index = new Index();
    private final Intake m_intake = new Intake();
    private final Pivot m_pivot = new Pivot();
    private final Hood m_hood = new Hood(m_vision);
    private final Climb m_climb = new Climb();

    // Autonomous
    private final SendableChooser<Command> autoChooser;
    private final Field2d m_field = new Field2d();

    // Drive parameters
    private final double maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    private final double maxAngularRate = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    // Slew rate limiters for acceleration-only limiting
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private double lastXVelocity = 0.0;
    private double lastYVelocity = 0.0;

    private double accelOnlyLimit(double target, double lastValue, SlewRateLimiter limiter) {
        if (Math.abs(target) < Math.abs(lastValue) || (target * lastValue < 0 && target != 0)) {
            limiter.reset(target);
            return target;
        }
        return limiter.calculate(target);
    }

    // Swerve requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric limelight = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Controllers
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        // Wire up safety interlocks and dependencies
        m_climb.setPivot(m_pivot);
        m_pivot.setClimb(m_climb);
        m_vision.setClimb(m_climb);

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        drivetrain.registerTelemetry(state -> m_field.setRobotPose(state.Pose));

        configureBindings();

        if (Utils.isSimulation()) {
            drivetrain.resetPose(new Pose2d(3.5052, 1.0668, Rotation2d.fromDegrees(0)));
        }
    }

    private void registerNamedCommands() {
        // Intake pivot commands
        NamedCommands.registerCommand("LowerIntake", m_pivot.lowerArmManual(PivotConstants.PIVOT_SPEED));
        NamedCommands.registerCommand("RaiseIntake", m_pivot.raiseArmManual(PivotConstants.PIVOT_SPEED));

        // Climb commands
        NamedCommands.registerCommand("LowerClimb", m_climb.runClimbDown());
        NamedCommands.registerCommand("RaiseClimb", m_climb.runClimbUp());

        // Intake roller command
        NamedCommands.registerCommand("runIntake", m_intake.runIntake(IntakeConstants.INTAKE_SPEED));
        NamedCommands.registerCommand("stopIntake", m_intake.stopAll());

        // Reverse feeder
        NamedCommands.registerCommand("reverseFeeder", m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED));

        // Shoot command (spins up shooter, then feeds when at speed)
        NamedCommands.registerCommand("Shoot", PIDShooter_Feeder_Index());
    }

    private void configureBindings() {
        configureDrivetrainBindings();
        configureShooterBindings();
        configureIntakeBindings();
        configureDefaultCommands();
    }

    private void configureDrivetrainBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double targetX = MathUtil.applyDeadband(-m_driverController.getLeftY() * maxSpeed, 0.15);
                double targetY = MathUtil.applyDeadband(-m_driverController.getLeftX() * maxSpeed, 0.15);
                lastXVelocity = accelOnlyLimit(targetX, lastXVelocity, xLimiter);
                lastYVelocity = accelOnlyLimit(targetY, lastYVelocity, yLimiter);
                return drive
                    .withVelocityX(lastXVelocity)
                    .withVelocityY(lastYVelocity)
                    .withRotationalRate(MathUtil.applyDeadband(-m_driverController.getRightX() * maxAngularRate, 0.15));
            })
        );

        // A Button: Auto-aim to goal using odometry
        m_driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * maxSpeed * 0.75)
                    .withVelocityY(-MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed * 0.75)
                    .withRotationalRate(m_vision.getRotationToGoal())
            )
        );

        // Start Button: Reset gyro heading
        m_driverController.start().onTrue(new InstantCommand(() ->
            drivetrain.resetPose(new Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                new Rotation2d()
            ))
        ));

        // Left Bumper: Brake (X-pattern wheel lock)
        m_driverController.leftBumper().whileTrue(drivetrain.brakeCommand());
    }

    private void configureShooterBindings() {
        // Y Button: Override mode toggle
        m_driverController.y()
            .onTrue(new InstantCommand(() -> Constants.overrideEnabled = true))
            .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));

        // Right Trigger: Auto-aim shooter with auto-feed (waits for speed)
        m_driverController.rightTrigger().whileTrue(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                    m_index.runIndex(IndexConstants.INDEX_SPEED),
                    m_feeder.runFeeder(FeederConstants.FEEDER_SPEED)
                ),
                new SequentialCommandGroup(
                    m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
                        .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 5.0)),
                    new ParallelCommandGroup(
                        m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
                        m_index.runIndex(-IndexConstants.INDEX_SPEED),
                        m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
                    )
                ),
                () -> Constants.overrideEnabled
            )
        );

        // Right Bumper: Shooter only
        m_driverController.rightBumper().whileTrue(
            new ConditionalCommand(
                m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                () -> Constants.overrideEnabled
            )
        );

        // Left Trigger: Run intake rollers
        m_driverController.leftTrigger().whileTrue(
            new ConditionalCommand(
                m_intake.runIntake(-IntakeConstants.INTAKE_SPEED),
                m_intake.runIntake(IntakeConstants.INTAKE_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // B Button: Feeder only
        m_driverController.b().whileTrue(
            new ConditionalCommand(
                m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // X Button: Toggle index belt
        m_driverController.x().toggleOnTrue(
            new ConditionalCommand(
                m_index.runIndex(1),
                new ParallelCommandGroup(
                    m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                    m_index.runIndex(-1)
                ),
                () -> Constants.overrideEnabled
            )
        );

        // D-Pad Left/Right: Hood control
        m_driverController.povLeft().whileTrue(m_hood.runHood(0.1));
        m_driverController.povRight().whileTrue(m_hood.runHood(-0.1));
    }

    private void configureIntakeBindings() {
        // D-Pad Up: Raise intake arm (or climb up in override mode)
        m_driverController.povUp().whileTrue(
            new ConditionalCommand(
                m_climb.runClimbUp(),
                m_pivot.raiseArmManual(PivotConstants.PIVOT_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // D-Pad Down: Lower intake arm (or climb down in override mode)
        m_driverController.povDown().whileTrue(
            new ConditionalCommand(
                m_climb.runClimbDown(),
                m_pivot.lowerArmManual(PivotConstants.PIVOT_SPEED),
                () -> Constants.overrideEnabled
            )
        );

    }

    private void configureDefaultCommands() {
        m_shooter.setDefaultCommand(m_shooter.stopAll());
        m_feeder.setDefaultCommand(m_feeder.stopAll());
        m_intake.setDefaultCommand(m_intake.stopAll());
        m_pivot.setDefaultCommand(m_pivot.stopAll());
        m_index.setDefaultCommand(m_index.stopAll());
        m_climb.setDefaultCommand(m_climb.stopAll());
        m_hood.setDefaultCommand(m_hood.stopAll());

        // Set initial pose from vision at start of auto and teleop
        RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> m_vision.setInitialPoseFromVision()));
        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> m_vision.setInitialPoseFromVision()));

        // Calibrate subsystems at start of auto (defer creates fresh command each time)
        RobotModeTriggers.autonomous().onTrue(Commands.defer(m_hood::calibrateHood, Set.of(m_hood)).unless(m_hood::isCalibrated));
        RobotModeTriggers.autonomous().onTrue(Commands.defer(m_pivot::calibratePivot, Set.of(m_pivot)).unless(m_pivot::isCalibrated));
        RobotModeTriggers.autonomous().onTrue(Commands.defer(m_climb::calibrateClimb, Set.of(m_climb)).unless(m_climb::isCalibrated));

        // Calibrate subsystems on teleop start if not already calibrated
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_hood::calibrateHood, Set.of(m_hood)).unless(m_hood::isCalibrated));
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_pivot::calibratePivot, Set.of(m_pivot)).unless(m_pivot::isCalibrated));
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_climb::calibrateClimb, Set.of(m_climb)).unless(m_climb::isCalibrated));

    
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Creates a command that spins up the shooter, then feeds when at speed. */
    public Command PIDShooter_Feeder_Index() {
        return new SequentialCommandGroup(
                m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
                    .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 5.0)),
            new ParallelCommandGroup(
                m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
                m_index.runIndex(IndexConstants.INDEX_SPEED),
                m_feeder.runFeeder(FeederConstants.FEEDER_SPEED)
            )
        );
    }
}
