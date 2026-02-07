package frc.robot.subsystems.Drive;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.LimelightHelpers;
import com.revrobotics.spark.SparkFlex;


//     private final SparkFlex hoodMotor = new SparkFlex(24, MotorType.kBrushless); // Neo brushless vortex
//     private final RelativeEncoder encoder = new RelativeEncoder();
//     private DigitalInput topSwitch = new DigitalInput(0);
//     private DigitalInput bottomSwitch = new DigitalInput(0);
//     double ta = LimelightHelpers.getTY("limelight-four");
public class Hood {
    private final SparkFlex hoodMotor = new SparkFlex(24, MotorType.kBrushless); // Neo brushless vortex
    private final RelativeEncoder encoder = new RelativeEncoder();
    double ta = LimelightHelpers.getTY("limelight-four");

    /*
     * return the distance between 
     * shooter and the target
     */
    public double distance() {

    }

    /*
     * use the distance and height of the target to find the necessary angle of the hood
    */
    public double angleFinder(double Distance) {
        int speed = 0;
        
        return speed;
    }

    /*
     * adjust the motor to rotate the hood until it hits the right angle
     */
    public Command moveHood(double speed) {

    }
    



    // will the bot ever B shooting while it's moving??????????
}
