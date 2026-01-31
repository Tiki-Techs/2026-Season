// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SlowDriveTrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Create Drivetrain from generated constants
    private final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();
    private final Vision m_Vision = new Vision(drivetrain);
    private final SlowDriveTrain m_SlowDriveTrain = new SlowDriveTrain();
    private final Shooter m_shooter = new Shooter();
    private final SendableChooser<Command> autoChooser;



    // Speed limits for drive
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Slew rate limiters to control acceleration (units per second)
    // Lower values = slower acceleration, higher values = snappier response
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);  // 3 m/s per second
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);  // 3 m/s per second
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0); // 3 rad/s per second

    // Field oriented drive request
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // Brake request
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // Point wheels request
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // Limelight assisted drive request - no deadband for precise positioning
    private final SwerveRequest.FieldCentric limelight = new SwerveRequest.FieldCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Default drive command with acceleration limiting
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(xLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), .15) * maxSpeed)) // forward/back
                    .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), .15) * maxSpeed)) // left/right
                    .withRotationalRate(rotLimiter.calculate(-MathUtil.applyDeadband(m_driverController.getRightX(), .15) * maxAngularRate)) // rotate
        )
    );
    m_driverController.leftBumper().whileTrue(
      drivetrain.applyRequest(
        () ->
            limelight
                .withVelocityX(xLimiter.calculate(-m_Vision.limelight_range_proportional()))
                .withVelocityY(yLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), .15) * maxSpeed))
                .withRotationalRate(m_Vision.limelight_aim_proportional())

      ));

      m_shooter.setDefaultCommand(
        m_shooter.stopAll()
    );

    m_driverController.rightTrigger().whileTrue(m_shooter.runShooterIntake(m_driverController));
    m_driverController.leftTrigger().whileTrue(m_shooter.runShooter(m_driverController));

    
    // Idle (coast) when x is held while disabled
    final var idle = new SwerveRequest.Idle();
    m_driverController.x().and(RobotModeTriggers.disabled())
      .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Limelight throttle: 150 when disabled, 0 when enabled
    RobotModeTriggers.disabled()
      .onTrue(Commands.runOnce(() -> LimelightHelpers.setLimelightNTDouble("limelight", "throttle", 150)));
    RobotModeTriggers.teleop().or(RobotModeTriggers.autonomous())
      .onTrue(Commands.runOnce(() -> LimelightHelpers.setLimelightNTDouble("limelight", "throttle", 0)));

    // Brake when a is pressed
    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // Point wheels when b is pressed
    m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
    point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
    ));

  }





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto();
    return autoChooser.getSelected();

  }
}
