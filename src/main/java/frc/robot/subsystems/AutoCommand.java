// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;


// public class AutoCommand {

//         // Create New Sendable Chooser for autonomous command selection on the dashboard
//     private final SendableChooser<Command> autoChooser;

//     public AutoCommand(){
//     NamedCommands.registerCommand("runShooter", RobotContainer.m_shooter.runShooter(1.0));

//     autoChooser = AutoBuilder.buildAutoChooser();
//     SmartDashboard.putData("Auto Chooser", autoChooser);
//     }

//      public Command getAutonomousCommand() {
//     // An example command will be run in autonomous
//     // return Autos.exampleAuto();
//     return autoChooser.getSelected();

//   }
// }

