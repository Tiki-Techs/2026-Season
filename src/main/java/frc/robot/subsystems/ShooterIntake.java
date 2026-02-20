package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterIntakeConstants;

public class ShooterIntake extends SubsystemBase{
    
    private final TalonFX shooterIntake = new TalonFX(ShooterIntakeConstants.SHOOTER_INTAKE);

    public Command runShooterIntake(double speed){
        return new RunCommand(()->{
            shooterIntake.set(speed);
                }, this);
    } 

    public Command stopShooterIntake(){
        return new RunCommand(()->{
            shooterIntake.set(0);
                }, this);
    } 

    public Command stopAll(){
        return new RunCommand(()->{
            shooterIntake.set(0);
                }, this);
    } 


}