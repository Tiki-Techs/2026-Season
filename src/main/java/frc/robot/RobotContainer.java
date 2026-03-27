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
import frc.robot.commands.SlowDriveTrain;
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
    private final Climb m_climb = new Climb();

    // Autonomous
    private final SendableChooser<Command> autoChooser;
    private final Field2d m_field = new Field2d();

    // Drive parameters
    private final double maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    private final double maxAngularRate = DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    // Swerve requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric limelight = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Slow drive command
    private final SlowDriveTrain slowDriveTrain = new SlowDriveTrain();

    // Controllers
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
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

        // Index 
        NamedCommands.registerCommand("runIndex", m_index.runIndex(IndexConstants.INDEX_SPEED));
        NamedCommands.registerCommand("stopIndex", m_index.stopAll());

        // Reverse feeder
        NamedCommands.registerCommand("reverseFeeder", m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED));
        // Shoot command (spins up shooter, then feeds when at speed)
        NamedCommands.registerCommand("Shoot", PIDShooter_Feeder_Index());


        // Calibration commands
        NamedCommands.registerCommand("calibratePivot", m_pivot.calibratePivot());
        NamedCommands.registerCommand("calibrateClimb", m_climb.calibrateClimb());
    }

    private void configureBindings() {
        // Y Button: Override mode toggle
        m_driverController.y()
            .onTrue(new InstantCommand(() -> Constants.overrideEnabled = true))
            .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));

        configureDrivetrainBindings();
        configureShooterBindings();
        configureIntakeBindings();
        configureClimbBindings();
        configureDefaultCommands();
        configureOperatorBindings();

    }

    private void configureDrivetrainBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15) * maxSpeed)
                .withVelocityY(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15) * maxSpeed)
                .withRotationalRate(MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15) * maxAngularRate)
            )
        );

        // Configure PID for auto-aim rotation controller
        autoAim.HeadingController.setPID(10.0, 0.0, 0.1);
        autoAim.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Right Bumper: Auto-aim to goal with lookahead
        m_driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> {
            var state = drivetrain.getState();

            // 1. Get raw inputs from controller (reduced to 50% for better control while aiming)
            double vx = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * 0.5 * maxSpeed;
            double vy = -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * 0.5 * maxSpeed;

            /* 2. POSE PREDICTION (Lookahead)
             * Predict where the robot will be in 50ms to compensate for
             * latency and the robot's own momentum.
             */
            double lookaheadSeconds = 0.050;
            Pose2d futurePose = new Pose2d(
                state.Pose.getX() + (state.Speeds.vxMetersPerSecond * lookaheadSeconds),
                state.Pose.getY() + (state.Speeds.vyMetersPerSecond * lookaheadSeconds),
                state.Pose.getRotation()
            );

            // 3. Calculate target angle using the predicted pose
            Rotation2d targetAngle = FieldAiming.getAngleToHub(futurePose);

            // Add 180° offset for red alliance (robot faces opposite direction)
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                targetAngle = targetAngle.plus(Rotation2d.fromDegrees(180));
            }

            double distance = FieldAiming.getDistanceToHub(state.Pose);

            // 4. Apply Request with FieldCentricFacingAngle for smooth rotation
            return autoAim
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(targetAngle);
        }));

        
        // X: Reset heading
        m_driverController.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // B: Brake (X-pattern wheel lock)
        m_driverController.b().whileTrue(drivetrain.brakeCommand());

        // A: Slow drive mode
        m_driverController.a().whileTrue(
            slowDriveTrain.slowDown(drivetrain, maxSpeed, maxAngularRate, m_driverController)
        );

        }
        
        
        private void configureShooterBindings() {
                
            // Right Trigger: Flow
            m_driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                        m_index.runIndex(-IndexConstants.INDEX_SPEED),
                        m_feeder.runFeeder(FeederConstants.FEEDER_SPEED)
                    ),
                    new SequentialCommandGroup(
                        m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
                            .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 5.0)),
                        new ParallelCommandGroup(
                            m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
                            m_index.runIndex(IndexConstants.INDEX_SPEED),
                            m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
                            )
                    ),
                    () -> Constants.overrideEnabled
                )
            );
                                                                                     
            // X Button: Toggle index belt and reverse feeder
            m_driverController.leftBumper().toggleOnTrue(
                new ConditionalCommand(
                    m_index.runIndex(1),
                    new ParallelCommandGroup(
                    m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                    m_index.runIndex(1)
                    ),
                    () -> Constants.overrideEnabled
                )
            );
    }
                                                        
    private void configureIntakeBindings() {

        // Left Trigger: Run intake rollers
        m_driverController.leftTrigger().whileTrue(
            new ConditionalCommand(
                m_intake.runIntake(-IntakeConstants.INTAKE_SPEED),
                m_intake.runIntake(IntakeConstants.INTAKE_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // D-pad Up: Pivot up
        m_driverController.povUp().whileTrue(
            m_pivot.runPivot(1.0)
        );
        // D-pad Down: Pivot down        
        m_driverController.povDown().whileTrue(
            m_pivot.runPivot(-1.0)
        );
                                   
    }

    private void configureClimbBindings() {
        // Climb controls
        m_driverController.povRight().whileTrue(m_climb.runClimbUp());
        m_driverController.povLeft().whileTrue(m_climb.runClimbDown());
    }
                    
    private void configureDefaultCommands() {
        m_shooter.setDefaultCommand(m_shooter.stopAll());
        m_feeder.setDefaultCommand(m_feeder.stopAll());
        m_intake.setDefaultCommand(m_intake.stopAll());
        m_pivot.setDefaultCommand(m_pivot.stopAll());
        m_index.setDefaultCommand(m_index.stopAll());
        m_climb.setDefaultCommand(m_climb.stopAll());

        // Configure Limelight 4 IMU modes for better pose estimation
        // Mode 1 (seeding) during disabled - syncs internal IMU with Pigeon
        RobotModeTriggers.disabled().onTrue(new InstantCommand(() -> {
            frc.robot.LimelightHelpers.SetIMUMode("limelight-right", 1);
            frc.robot.LimelightHelpers.SetIMUMode("limelight-left", 1);
        }));

        // Mode 4 (internal + external assist) when enabled - uses LL4 1kHz IMU with gentle Pigeon correction
        RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> {
            frc.robot.LimelightHelpers.SetIMUMode("limelight-right", 4);
            frc.robot.LimelightHelpers.SetIMUMode("limelight-left", 4);
        }));

        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> {
            frc.robot.LimelightHelpers.SetIMUMode("limelight-right", 4);
            frc.robot.LimelightHelpers.SetIMUMode("limelight-left", 4);
        }));


        }

        private void configureOperatorBindings() {
            m_operatorController.leftTrigger().whileTrue(
                new ConditionalCommand(
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    () -> Constants.overrideEnabled
                )
            );


        // Right Bumper: Slow drive mode
        m_operatorController.rightBumper().whileTrue(
            slowDriveTrain.slowDown(drivetrain, maxSpeed, maxAngularRate, m_driverController)
        );
         
        // Left Bumper: Brake (X-pattern wheel lock)
        m_operatorController.leftBumper().whileTrue(drivetrain.brakeCommand());


        }



    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Creates a command that spins up the shooter, then feeds when at speed. */
    public Command PIDShooter_Feeder_Index() {
        return new SequentialCommandGroup(
                m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
                    .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 8.0)),
            new ParallelCommandGroup(
                m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
                m_index.runIndex(IndexConstants.INDEX_SPEED),
                m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
            )
        );
    }
}
