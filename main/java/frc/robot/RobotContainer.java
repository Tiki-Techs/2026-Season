// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Drive.Vision;
import frc.robot.commands.DriveToPointCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive.Shooter;
import frc.robot.subsystems.Drive.Intake;
import frc.robot.subsystems.Drive.Index;
// import frc.robot.subsystems.Drive.Climb;
import frc.robot.subsystems.Drive.Arm;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
//   private final TestFile m_TestFilep = new TestFile();
      private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
      private final Vision m_Vision = new Vision(); 
      private final Shooter m_shooter = new Shooter();
      private final Intake m_intake = new Intake();
      private final Index m_index = new Index();
      // private final RobotContainer m_robotContainer = new RobotContainer();
      // private final Climb m_climb = new Climb();
      private final Arm m_arm = new Arm();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.driveCommandF(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), .15),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), .15),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), .15)
            // () -> 0.0,
            // () -> 0.0,
            // () -> 0.0
        )
    );
    //   m_driverController.a().whileTrue(
    //     m_swerveSubsystem.driveCommandL(
    //       () -> MathUtil.applyDeadband(m_Vision.limelight_range_proportional(), .15),
    //       // () -> 0.0,
    //       () -> 0.0, 
    //       // () -> 0.0
    //       () -> MathUtil.applyDeadband(m_Vision.limelight_aim_proportional(), .15)
    //       )
    // );
      // make sure all subsystems have a default command to fall back upon when not being called
    m_shooter.setDefaultCommand(
      m_shooter.stopAll()
    );
    
    m_intake.setDefaultCommand(
      m_intake.stopAll()
    );

    m_index.setDefaultCommand(
      m_index.stopAll()
    );


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

    // X Button: Drive to a specific absolute field coordinate (Example: 5.0m X, 1.5m Y, facing 0 degrees)
    // m_driverController.x().onTrue(
    //     DriveToPointCommand.getCommand(
    //         new Pose2d(15.0, .65, Rotation2d.fromDegrees(0)), 
    //         m_swerveSubsystem
    //     ).andThen(m_swerveSubsystem::stop)
    // );

// working code for index runner
    // m_driverController.rightTrigger().whileTrue(
    //   m_index.runIndex(m_driverController)).whileFalse(m_index.stopIndex());
    // m_driverController.a().whileTrue(
    //   m_index.runReverseIndex(m_driverController)).whileFalse(m_index.stopIndex());


    // m_driverController.leftTrigger().whileTrue(
    //   m_shooter.runShooter(m_driverController)).whileFalse(m_shooter.stopShooter());
    // m_driverController.x().whileTrue(
    //    m_shooter.runShooterBangBang(50)).whileFalse(m_shooter.stopShooter());
    // m_driverController.x().whileTrue(
    //   m_shooter.runPIDShooter(60)).whileFalse(m_shooter.stopShooter());


    // B Button: Run Intake, press again to fall back on default commmand (stop intake)
    // m_driverController.b().toggleOnTrue(m_intake.runIntake());
    // m_driverController.a().toggleOnTrue(m_index.runIndex());
    
    // m_driverController.rightBumper().whileTrue(m_intake.raiseIntakeManual());
    // m_driverController.leftBumper().whileTrue(m_intake.lowerIntakeManual());

    // m_driverController.y().onTrue(m_intake.toggleIntake());
    // m_driverController.x().whileTrue(m_intake.runIntake());

    // test override commands
    m_driverController.start()
    .onTrue(new InstantCommand(()-> Constants.overrideEnabled = true))
    .onFalse(new InstantCommand(() -> Constants.overrideEnabled = false));

    // dpad binds
    m_driverController.povUp().whileTrue(
      new ConditionalCommand(
        m_arm.raiseArmManual(),  //  if override = true, run manual raise intake
        m_arm.raiseArmAuto(),  // if override = false, run auto raise intake
        ()-> Constants.overrideEnabled)
    );

    m_driverController.povDown().whileTrue(
      new ConditionalCommand(
        m_arm.lowerArmManual(),  //  if override = true, run manual raise intake
        m_arm.lowerArmAuto(),  // if override = false, run auto raise intake
        ()-> Constants.overrideEnabled)
    );

    // m_driverController.povLeft();
    // m_driverController.povRight();

    // Trigger and bumper binds
    m_driverController.leftBumper().whileTrue(
      new ConditionalCommand(
       m_intake.runIntake(),
       m_intake.runIntake(),
      () -> Constants.overrideEnabled)
      );

    m_driverController.rightBumper().whileTrue(
      new ConditionalCommand(
       m_shooter.runShooter(-1),
       m_shooter.runPIDShooter(60),
      () -> Constants.overrideEnabled)
      );    
      
    m_driverController.leftTrigger().whileTrue(
      new ConditionalCommand(
        m_shooter.runReverseShooter(m_driverController),
        m_shooter.runShooter(m_driverController),
        () -> Constants.overrideEnabled)
      );

    m_driverController.rightTrigger().whileTrue(
      new ConditionalCommand(
        m_index.runReverseIndex(m_driverController),
        m_index.runIndex(m_driverController),
        () -> Constants.overrideEnabled)
      );

    // button binds
    m_driverController.b().toggleOnTrue(m_intake.runIntake());
    m_driverController.x().whileTrue(
      new ConditionalCommand(
       m_index.runIndex(-1),
       m_index.runIndex(1),
      () -> Constants.overrideEnabled)
      );

    // m_driverController.a();
    // m_driverController.y();


  // m_driverController.x().toggleOnTrue(m_index.runIndex(1));


  }
    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // }
  public void periodic(){
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_index);
    SmartDashboard.putData(m_arm);
  }
}

