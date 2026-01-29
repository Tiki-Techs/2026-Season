package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class SlowDriveTrain{
    
    public Command slowDown(SwerveSubsystem drivetrain, double CurrentMaxSpeed,
                             double currentMaxAngularRate, CommandXboxController joystick)
        {
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric(); 

            return drivetrain.applyRequest(() -> {
                double vscale = 0.1;
                double vrotation = 0.025;
               

                return drive
                    .withVelocityX(-joystick.getLeftY() * CurrentMaxSpeed * vscale) 
                    .withVelocityY(-joystick.getLeftX() * CurrentMaxSpeed * vscale) 
                    .withRotationalRate(-joystick.getRightX() * currentMaxAngularRate * vrotation); 
            });
        
    }
}
