package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Index extends SubsystemBase{
    // 3, 2x belt index
    // up/down movement on belt arms?
    // or (manually) adjustable belts, thats only runs?
    // do we need encoders/limit switches here
    private final TalonFX indexMotor = new TalonFX(25);

    public Command runIndex(CommandXboxController controllerValue) {
        return new RunCommand(() -> 
            indexMotor.set(controllerValue.getRightTriggerAxis())
            ,this
    );}
    
    public Command runReverseIndex(CommandXboxController controllerValue) {
        return new RunCommand(() -> 
            indexMotor.set(-controllerValue.getRightTriggerAxis())
            ,this
    );}

    public Command runIndex(double speed){
        return new RunCommand(() -> 
            indexMotor.set(speed)
            ,this
    );}


    public Command stopIndex(){
        return new InstantCommand(()->
            indexMotor.set(0)
            ,this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
                indexMotor.set(0);
            },
            // ", this" makes sure that only the shooter subsystem object can only run command at a time
            this
        );
    }

@Override
public void periodic() {

}   
} 