package frc.robot.subsystems;
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
    private final SparkFlex leaderIntake = new SparkFlex(25, MotorType.kBrushless); // Neo brushless vortex
    private boolean intakeDeployed = true; // true if the intake is currently deployed, false if it is currently stowed.


    // pivotArm.setIdleMode(IdleMode.kBrake);
    // private final SparkFlex pivotArm = new SparkFlex(28, MotorType.kBrushless);



    
    public Command runIntake(double speed){
        return new RunCommand(() -> {
            leaderIntake.set(-speed); // negative to make it spin the right direction, fix with motor configs later
            // followerIntake.set(-setSpeed);
        }
        , this 
        );
    }

    public Command runReverseIntake(double speed){
        return new RunCommand(() -> {
            leaderIntake.set(speed); // positive to make it spin the right direction, fix with motor configs later
            // followerIntake.set(-setSpeed);
        }
        , this 
        );
    }
    public Command stopIntake(){
        return new RunCommand(()->{
            leaderIntake.set(0);
            // followerIntake.set(stopSpeed);
        },
        this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
            leaderIntake.set(0);
            // followerIntake.set(stopSpeed);
        },
        this
        );
    }

    public Command changeDeployState(){
        return new InstantCommand(()->{
            intakeDeployed = !intakeDeployed;
        });
    }

    @Override
    public void periodic() {
     
    }
}