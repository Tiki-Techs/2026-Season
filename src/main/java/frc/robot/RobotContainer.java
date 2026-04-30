// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;


import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.CornerDumpConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PivotCommandAuto;
import frc.robot.commands.ShootCommandAuto;
import frc.robot.commands.autoclimb.AutoclimbCommand;
import frc.robot.commands.ShootCommandAutoCenter;
import frc.robot.commands.ShootCommandAutoLong;
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
    ShootCommandAutoCenter centerShootCommand = new ShootCommandAutoCenter(m_shooter, m_index, m_feeder, m_vision);
    ShootCommandAutoLong longShootAutoCommand = new ShootCommandAutoLong(m_shooter, m_index, m_feeder, m_vision);
    ShootCommandAuto shootCommandAuto = new ShootCommandAuto(m_shooter, m_index, m_feeder, m_vision);
    PivotCommandAuto pivotCommandAuto = new PivotCommandAuto(m_pivot);

    ParallelCommandGroup intakeCommandParallel = new ParallelCommandGroup(m_intake.runIntake(IntakeConstants.INTAKE_SPEED), m_index.runIndex(1.0));


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

    private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Separate swerve request for corner dump auto-aim (same config, distinct instance)
    private final SwerveRequest.FieldCentricFacingAngle cornerDumpAutoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

        NamedCommands.registerCommand("ShootCommandAuto", shootCommandAuto);
        NamedCommands.registerCommand("PivotCommandAuto", pivotCommandAuto);    
        NamedCommands.registerCommand("CenterShootCommand", centerShootCommand);    
        NamedCommands.registerCommand("IntakeCommand", intakeCommandParallel);    
        NamedCommands.registerCommand("ShootCommandAutoLong", longShootAutoCommand);



        // Intake pivot commands
        NamedCommands.registerCommand("LowerIntake", m_pivot.lowerArmManual(PivotConstants.PIVOT_SPEED));
        NamedCommands.registerCommand("RaiseIntake", m_pivot.raiseArmManual(PivotConstants.PIVOT_SPEED));

        // Climb commands
        NamedCommands.registerCommand("LowerClimb", m_climb.runClimbDown());
        NamedCommands.registerCommand("RaiseClimb", m_climb.runClimbUp());

        // Intake roller command
        NamedCommands.registerCommand("runIntake", m_intake.runIntake(-IntakeConstants.INTAKE_SPEED));
        NamedCommands.registerCommand("stopIntake", m_intake.stopAll());

        // Index 
        NamedCommands.registerCommand("runIndex", m_index.runIndex(IndexConstants.INDEX_SPEED));
        NamedCommands.registerCommand("stopIndex", m_index.stopAll());

        // Reverse feeder
        NamedCommands.registerCommand("reverseFeeder", m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED));
        
        // Stops feeder
        NamedCommands.registerCommand("stopFeeder", m_feeder.stopAll());

        // Shoot command (spins up shooter, then feeds when at speed)
        NamedCommands.registerCommand("Shoot", PIDShooter_Feeder_Index());
        NamedCommands.registerCommand("windUpShooter", windUpShooter());
        NamedCommands.registerCommand("stopShoot", Stop_PIDShooter_Feeder_Index());



        // Calibration commands
        NamedCommands.registerCommand("calibratePivot", m_pivot.calibratePivot());
        NamedCommands.registerCommand("calibrateClimb", m_climb.calibrateClimb());

        // Autoclimb — full autonomous cage-climb sequence
        NamedCommands.registerCommand("autoclimb",
            new AutoclimbCommand(drivetrain, m_climb, m_vision)
                .finallyDo((interrupted) -> {
                    if (interrupted) m_climb.emergencyRetract().schedule();
                    m_vision.resumeFusedMode();
                })
        );
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

        // Configure PID for corner dump auto-aim rotation controller (same gains)
        cornerDumpAutoAim.HeadingController.setPID(10.0, 0.0, 0.1);
        cornerDumpAutoAim.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Left Bumper: Auto-aim to goal with lookahead
        m_driverController.leftBumper().whileTrue(drivetrain.applyRequest(() -> {
            var state = drivetrain.getState();

            // 1. Get raw inputs from controller (reduced speed for better control while aiming)
            double vx = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;
            double vy = -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;

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
            

            // 4. Apply Request with FieldCentricFacingAngle for smooth rotation
            return autoAim
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(targetAngle);
        }));

        
        // Back (⊟): Reset heading  (X reassigned to autoclimb)
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.getPigeon2().setYaw(0)));

        // B: Brake (X-pattern wheel lock)
        m_driverController.b().whileTrue(drivetrain.brakeCommand());

}
        
        
        private void configureShooterBindings() {
                
            // Right Trigger: Flow
            m_driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        m_shooter.runPIDShooter(ShooterConstants.SHOOTER_TARGET_RPS),
                        m_index.runIndex(-IndexConstants.INDEX_SPEED),
                        m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                        new SequentialCommandGroup(
                            new edu.wpi.first.wpilibj2.command.WaitCommand(CornerDumpConstants.INTAKE_ASSIST_DELAY_S),
                            m_pivot.raiseToIntakeAssistPosition()
                        )
                    ),
                    new SequentialCommandGroup(
                        m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal())
                            .until(() -> m_shooter.isAtAutoAimTargetSpeed(m_vision.getDistanceToGoal(), 5.0)),
                        new ParallelCommandGroup(
                            m_shooter.autoAimShooter(() -> m_vision.getDistanceToGoal()),
                            m_index.runIndex(IndexConstants.INDEX_SPEED),
                            m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED),
                            new SequentialCommandGroup(
                                new edu.wpi.first.wpilibj2.command.WaitCommand(CornerDumpConstants.INTAKE_ASSIST_DELAY_S),
                                m_pivot.raiseToIntakeAssistPosition()
                            )
                        )
                    ),
                    () -> Constants.overrideEnabled
                )
            );
            // When right trigger released, lower pivot back down fast
            m_driverController.rightTrigger().onFalse(
                m_pivot.lowerToBottom().until(m_pivot::isAtLowerLimit).withTimeout(3.0)
            );

            // Start Button: Fixed speed shooting (waits until at speed before feeding)
            m_driverController.start().whileTrue(
                new SequentialCommandGroup(
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS)
                        .until(() -> m_shooter.isAtTargetSpeed(-ShooterConstants.SHOOTER_TARGET_RPS, 5.0)),
                    new ParallelCommandGroup(
                        m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                        m_index.runIndex(IndexConstants.INDEX_SPEED),
                        m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
                    )
                )
            );

            // A Button: Test RPS — runs shooter/index/feeder at the test speed from Constants
            m_driverController.a().whileTrue(
                new ParallelCommandGroup(
                    m_shooter.runTestRPS(),
                    m_index.runIndex(IndexConstants.INDEX_SPEED),
                    m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
                )
            );

            // Right Bumper: Corner dump — auto-aims toward the nearest corner of our side of the
            // field (avoiding the hub), spins up shooter to distance-based speed, waits until aimed,
            // then feeds. 0.5s after feeding starts, raises pivot 3/4 up and runs intake to push
            // any balls stuck in the pivot/intake into the hopper.
            m_driverController.rightBumper().whileTrue(cornerDump());
            // When right bumper released, lower pivot back down fast
            m_driverController.rightBumper().onFalse(
                m_pivot.lowerToBottom().until(m_pivot::isAtLowerLimit).withTimeout(3.0)
            );


        
    }
                                                        
    private void configureIntakeBindings() {

        // Left Trigger: Run intake rollers
        m_driverController.leftTrigger().whileTrue(
            new ConditionalCommand(
                m_intake.runIntake(IntakeConstants.INTAKE_SPEED),
                m_intake.runIntake(-IntakeConstants.INTAKE_SPEED),
                () -> Constants.overrideEnabled
            )
        );

        // D-pad Up: Pivot up
        m_driverController.povUp().whileTrue(
            m_pivot.runPivot(-1.0)
        );
        // D-pad Down: Pivot down
        m_driverController.povDown().whileTrue(
            m_pivot.runPivot(1.0)
        );
                                   
    }

    private void configureClimbBindings() {
        // Manual climb controls
        m_driverController.povRight().whileTrue(m_climb.runClimbUp());
        m_driverController.povLeft().whileTrue(m_climb.runClimbDown());


        // X: Autoclimb — hold to run, release to abort
        m_driverController.x().whileTrue(
            new AutoclimbCommand(drivetrain, m_climb, m_vision)
                .finallyDo((interrupted) -> {
                    if (interrupted) m_climb.emergencyRetract().schedule();
                    m_vision.resumeFusedMode();
                })
        );
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
            frc.robot.LimelightHelpers.SetIMUMode("limelight-pivot", 4);
        }));

        RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> {
            frc.robot.LimelightHelpers.SetIMUMode("limelight-right", 4);
            frc.robot.LimelightHelpers.SetIMUMode("limelight-left", 4);
            frc.robot.LimelightHelpers.SetIMUMode("limelight-pivot", 4);
        }));

        // Auto-calibrate pivot on first teleop enable only.
        // Climb is excluded — it has hardware limit switches and can be calibrated manually via A button.
        RobotModeTriggers.teleop().onTrue(
            m_pivot.calibratePivot().unless(m_pivot::isCalibrated)
        );

        }

        private void configureOperatorBindings() {
            m_operatorController.leftTrigger().whileTrue(
                new ConditionalCommand(
                    m_shooter.runPIDShooter(-40),
                    m_shooter.runPIDShooter(-ShooterConstants.SHOOTER_TARGET_RPS),
                    () -> Constants.overrideEnabled
                )
            );


        // Right Bumper: Dump Balls (index out, feeder/shooter reversed, pivot auto-raises)
        m_operatorController.rightBumper().whileTrue(
            new ParallelCommandGroup(
                m_shooter.runOpenLoop(0.3),
                m_index.runIndex(-IndexConstants.INDEX_SPEED),
                m_feeder.runFeeder(FeederConstants.FEEDER_SPEED),
                m_pivot.raiseToTop()
            )
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

    /** Creates a command that spins up the shooter, then feeds when at speed. */
    public Command Stop_PIDShooter_Feeder_Index() {
       
            return new ParallelCommandGroup(
                m_shooter.runPIDShooter(0),
                m_index.runIndex(0),
                m_feeder.runFeeder(0)
            );
        
    }
    // temp test command
     public Command windUpShooter() {
            return new ParallelCommandGroup(
                m_shooter.runPIDShooter(-30)
            );
        
    }

    /**
     * Corner dump command — one button does everything:
     *
     * 1. Spins up shooter to distance-based speed for the selected corner target.
     * 2. Auto-aims the drivetrain toward the nearest corner of our alliance's side
     *    (audience or scoring side chosen by robot Y position). Full drive stick is
     *    still available at 50% speed so the driver can reposition while aiming.
     * 3. Waits until the heading is within HEADING_TOLERANCE_DEGREES of the corner.
     * 4. Starts feeding (index + feeder) once aimed.
     * 5. 0.5s after feeding starts, raises pivot halfway and runs intake to flush
     *    any balls stuck in the pivot/intake into the hopper.
     * 6. When the button is released, everything stops and the pivot returns to its
     *    original position automatically (default commands take over).
     *
     * Tune corner positions in CornerDumpConstants. Tune shooter speeds in
     * Shooter.distanceToCornerDumpSpeed.
     */
    public Command cornerDump() {

        // Phase 1: spin up shooter and auto-aim simultaneously. Ends when heading is close enough.
        // The shooter runs in parallel with the aim check so it's already spinning when feeding starts.
        Command aimUntilReady = new edu.wpi.first.wpilibj2.command.FunctionalCommand(
            () -> {},
            () -> {
                var state = drivetrain.getState();
                Pose2d currentPose = state.Pose;

                double vx = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;
                double vy = -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;

                Rotation2d targetAngle = FieldAiming.getAngleToCorner(currentPose);

                drivetrain.setControl(
                    cornerDumpAutoAim
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(targetAngle)
                );
            },
            (interrupted) -> {},
            () -> {
                Pose2d pose = drivetrain.getState().Pose;
                Rotation2d targetAngle = FieldAiming.getAngleToCorner(pose);
                double headingErrorDeg = Math.abs(
                    targetAngle.minus(pose.getRotation()).getDegrees());
                boolean headingOk = headingErrorDeg < CornerDumpConstants.HEADING_TOLERANCE_DEGREES;
                boolean speedOk = m_shooter.isAtCornerDumpTargetSpeed(
                    FieldAiming.getDistanceToCorner(pose),
                    CornerDumpConstants.SHOOTER_SPEED_TOLERANCE);
                return headingOk && speedOk;
            },
            drivetrain
        );

        // Use raceWith so the group ends as soon as aimUntilReady finishes (both conditions met).
        // ParallelCommandGroup would wait for ALL to finish — but cornerDumpShooter is a RunCommand
        // that never ends, so Phase 2 would never start.
        Command spinUpAndAim = aimUntilReady.raceWith(
            m_shooter.cornerDumpShooter(() -> FieldAiming.getDistanceToCorner(drivetrain.getState().Pose))
        );

        // Phase 2: aimed — feed balls while continuing to auto-aim and run shooter.
        // After INTAKE_ASSIST_DELAY_S seconds, also raise pivot halfway and run intake.
        Command feedAndAssist = new ParallelCommandGroup(
            // Continuous auto-aim + drive (same as phase 1 but now feeding)
            drivetrain.applyRequest(() -> {
                Pose2d currentPose = drivetrain.getState().Pose;
                double vx = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;
                double vy = -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15) * DriveConstants.SLOW_DRIVE_MULTIPLIER * maxSpeed;
                Rotation2d targetAngle = FieldAiming.getAngleToCorner(currentPose);
                return cornerDumpAutoAim
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withTargetDirection(targetAngle);
            }),
            // Distance-based shooter speed, continuously updated
            m_shooter.cornerDumpShooter(() -> FieldAiming.getDistanceToCorner(drivetrain.getState().Pose)),
            // Feed index and feeder
            m_index.runIndex(IndexConstants.INDEX_SPEED),
            m_feeder.runFeeder(-FeederConstants.FEEDER_SPEED),
            // After the delay, raise pivot halfway and run intake to flush stuck balls
            new SequentialCommandGroup(
                new edu.wpi.first.wpilibj2.command.WaitCommand(CornerDumpConstants.INTAKE_ASSIST_DELAY_S),
                new ParallelCommandGroup(
                    // Raise pivot to halfway point and hold — returns automatically when
                    // button is released and default stopAll() command takes over
                    m_pivot.raiseToIntakeAssistPosition(),
                    // Run intake to flush balls into hopper
                    m_intake.runIntake(-IntakeConstants.INTAKE_SPEED)
                )
            )
        );

        return new SequentialCommandGroup(spinUpAndAim, feedAndAssist);
    }
}
