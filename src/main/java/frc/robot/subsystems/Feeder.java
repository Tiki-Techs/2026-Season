package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{

    private final TalonFX feeder = new TalonFX(FeederConstants.FEEDER);


    // NEGATIVE runFeeder RUNS THE FEEDER IN THE CORRECT DIRECTION
    public Command runFeeder(double speed){
        return new RunCommand(()->{
            feeder.set(speed);
                }, this);
    }

    public Command stopFeeder(){
        return new RunCommand(()->{
            feeder.set(0);
                }, this);
    }

    public Command stopAll(){
        return new RunCommand(()->{
            feeder.set(0);
                }, this);
    }


}
