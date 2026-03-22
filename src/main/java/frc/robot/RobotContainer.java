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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final Hood m_hood = new Hood(m_vision);
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
        NamedCommands.registerCommand("calibrateHood", m_hood.calibrateHood());
        NamedCommands.registerCommand("calibratePivot", m_pivot.calibratePivot());  
        NamedCommands.registerCommand("calibrateClimb", m_climb.calibrateClimb());
    }

    private void configureBindings() {
        configureDrivetrainBindings();
        configureShooterBindings();
        configureIntakeBindings();
        configureDefaultCommands();
    }

    private void configureDrivetrainBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15) * maxSpeed)
                .withVelocityY(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15) * maxSpeed)
                .withRotationalRate(MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15) * maxAngularRate)
            )
        );

        // A Button: Auto-aim to goal
        // TWO OPTIONS - Comment/uncomment the one you want to use:

        // OPTION 2: Odometry-based PID (uses pose, may drift)
        
        m_driverController.a().whileTrue(
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * maxSpeed * 0.75)
                    .withVelocityY(-MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed * 0.75)
                    .withRotationalRate(m_vision.getRotationToGoal())
            )
        );
        

        // OPTION 3: TX-based aiming (direct camera, works with bad pose!)
        // m_driverController.a().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         drive
        //             .withVelocityX(-MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * maxSpeed * 0.75)
        //             .withVelocityY(-MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed * 0.75)
        //             .withRotationalRate(m_vision.getRotationToGoalTX())
        //     )
        // );

        // Start Button: Reset gyro heading
        m_driverController.start().onTrue(new InstantCommand(() ->
            drivetrain.resetPose(new Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                new Rotation2d()
            ))
        ));

        
        // Right Bumper: Slow drive mode
        m_operatorController.rightBumper().whileTrue(
            slowDriveTrain.slowDown(drivetrain, maxSpeed, maxAngularRate, m_driverController)
            );
            
            m_operatorController.leftBumper().whileTrue(drivetrain.brakeCommand());
            
        }
        
        
        private void configureShooterBindings() {
            // Y Button: Override mode toggle
            m_driverController.y()
            .onTrue(new InstantCommand(() -> Constants.overrideEnabled = true))
            .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));
            
            m_operatorController.leftTrigger().whileTrue(
                new ConditionalCommand(
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    () -> Constants.overrideEnabled
                )
            );
                    
            // Right Trigger: Auto-aim shooter with auto-feed (waits for speed)

            // OPTION 1: Shooter only (no auto-rotation)
            // m_driverController.rightTrigger().whileTrue(
            //     new ConditionalCommand(
            //         new ParallelCommandGroup(
            //             m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
            //             m_index.runIndex(-IndexConstants.INDEX_SPEED),
            //             m_feeder.runFeeder(FeederConstants.FEEDER_SPEED)
            //         ),
            //         new SequentialCommandGroup(
            //             m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
            //                 .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 5.0)),
            //             new ParallelCommandGroup(
            //                 m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
            //                 m_index.runIndex(IndexConstants.INDEX_SPEED),
            //                 m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
            //                 )
            //         ),
            //         () -> Constants.overrideEnabled
            //     )
            // );

            // OPTION 2: Shooter + auto-rotation (comment out OPTION 1 above and uncomment below)
            m_driverController.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                    // Auto-rotate drivetrain to goal
                    drivetrain.applyRequest(() ->
                        drive
                            .withVelocityX(-MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * maxSpeed * 0.75)
                            .withVelocityY(-MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * maxSpeed * 0.75)
                            .withRotationalRate(m_vision.getRotationToGoal())
                    ),
                    // Shooter/feeder/index functionality
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
                )
            );
                                                
                                                
            // X Button: Toggle index belt
            m_driverController.x().toggleOnTrue(
                new ConditionalCommand(
                    m_index.runIndex(1),
                    new ParallelCommandGroup(
                    m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                    m_index.runIndex(1)
                    ),
                    () -> Constants.overrideEnabled
            )
        );
            
                                                            
        // D-Pad Left/Right: Hood control
        m_driverController.povLeft().whileTrue(m_hood.runHood(0.1));
        m_driverController.povRight().whileTrue(m_hood.runHood(-0.1));
    }
                                                        
    private void configureIntakeBindings() {

        // Left Trigger: Run intake rollers + slow drive
        m_driverController.leftTrigger().whileTrue(
            new ParallelCommandGroup(
                slowDriveTrain.slowDown(drivetrain, maxSpeed, maxAngularRate, m_driverController),
                new ConditionalCommand(
                    m_intake.runIntake(-IntakeConstants.INTAKE_SPEED),
                    m_intake.runIntake(IntakeConstants.INTAKE_SPEED),
                    () -> Constants.overrideEnabled
                )
            )
        );

        m_driverController.povUp().whileTrue(
            new ConditionalCommand(
                m_pivot.runPivot(.1),
                m_pivot.runPivot(-1.0),
                () -> Constants.overrideEnabled
                )
                );
                
        m_driverController.povDown().whileTrue(
            new ConditionalCommand(
                m_pivot.runPivot(.1),                
                m_pivot.runPivot(.5),
                () -> Constants.overrideEnabled
                )
            );

        m_operatorController.povUp().toggleOnTrue(
            m_climb.getReady()
        );

        m_operatorController.povDown().toggleOnTrue(
            m_climb.climb()
        );
                        
        // Left Bumper: Brake (X-pattern wheel lock)
        m_driverController.leftBumper().whileTrue(m_climb.runClimbDown());
                
        // Right Bumper: Climb up
        m_driverController.rightBumper().whileTrue(m_climb.runClimbUp());
                       
    }
                    
    private void configureDefaultCommands() {
        m_shooter.setDefaultCommand(m_shooter.stopAll());
        m_feeder.setDefaultCommand(m_feeder.stopAll());
        m_intake.setDefaultCommand(m_intake.stopAll());
        m_pivot.setDefaultCommand(m_pivot.stopAll());
        m_index.setDefaultCommand(m_index.stopAll());
        m_climb.setDefaultCommand(m_climb.stopAll());
        m_hood.setDefaultCommand(m_hood.stopAll());

        // Calibration at start of auto is handled in getAutonomousCommand() to ensure
        // the auto routine waits for calibration to finish before running.

        // Calibrate subsystems on teleop start if not already calibrated
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_hood::calibrateHood, Set.of(m_hood)).unless(m_hood::isCalibrated));
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_pivot::calibratePivot, Set.of(m_pivot)).unless(m_pivot::isCalibrated));
        RobotModeTriggers.teleop().onTrue(Commands.defer(m_climb::calibrateClimb, Set.of(m_climb)).unless(m_climb::isCalibrated));

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
