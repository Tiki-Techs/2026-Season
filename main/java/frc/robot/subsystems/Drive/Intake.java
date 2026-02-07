package frc.robot.subsystems.Drive;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkFlex; // For the SPARK Flex controller
import com.revrobotics.spark.SparkMax;  // For the SPARK MAX controller (if using a solo adapter)
import com.revrobotics.spark.SparkLowLevel.MotorType; // To specify brushless motor type
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController; // For advanced control like PID loops (optional)
import com.revrobotics.spark.SparkRelativeEncoder; // For accessing the built-in encoder (optional)
import edu.wpi.first.wpilibj.DigitalInput;


import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;



public class Intake extends SubsystemBase{
    
    // private final TalonFX leaderIntake = new TalonFX(24);
    // private final TalonFX followerIntake = new TalonFX(25);
    private final SparkFlex leaderIntake = new SparkFlex(24, MotorType.kBrushless); // Neo brushless vortex
    


    // pivotArm.setIdleMode(IdleMode.kBrake);
    // private final SparkFlex pivotArm = new SparkFlex(28, MotorType.kBrushless);

    private final double stopSpeed = 0.0;
    private final double setSpeed = 0.1;

    
    public Command runIntake(){
        return new RunCommand(() -> {
            leaderIntake.set(setSpeed);
            // followerIntake.set(-setSpeed);
        }
        , this 
        );
    }

    public Command runReverseIntake(){
        return new RunCommand(() -> {
            leaderIntake.set(-setSpeed);
            // followerIntake.set(-setSpeed);
        }
        , this 
        );
    }


    public Command stopAll(){
        return new RunCommand(()->{
            leaderIntake.set(stopSpeed);
            // followerIntake.set(stopSpeed);
        },
        this
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putData("run intake", runIntake());
        SmartDashboard.putData("run reverse intake", runReverseIntake());

        // activating commands with a button test via SmartDashboard
        SmartDashboard.putData("run intake command", runIntake());
        SmartDashboard.putData("run reverse intake command", runReverseIntake());
    }
}
