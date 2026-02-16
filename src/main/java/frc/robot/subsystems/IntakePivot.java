package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase{


    // private final SparkMax leaderIntake = new SparkMax(24, MotorType.kBrushless);
    // private final SparkMax followerIntake = new SparkMax(25, MotorType.kBrushless);

    private final SparkMax pivotArm = new SparkMax(IntakePivotConstants.PIVOT_ARM, MotorType.kBrushless); // CanSpark Max with Neo brushless motor

    // when requesting a digital input, the boolean value will always be true if it is unplugged. 
    private final DigitalInput lowerLimitSwitch = new DigitalInput(IntakePivotConstants.LOWER_LIMIT_SWITCH);
    private final DigitalInput upperLimitSwitch = new DigitalInput(IntakePivotConstants.UPPER_LIMIT_SWITCH);

    // private final double pivotSpeed = 0.50; //this is MUCH faster than normal, normal was .25!!!!
    private final double stopSpeed = 0.0; // can add to constants. 

    private boolean intakeDeployed = true; 

    public IntakePivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        pivotArm.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
     public Command stopAll(){
        return new RunCommand(()->{
            pivotArm.set(stopSpeed);
        }
        , this
        );
    }

    // Press and hold verision
    public Command lowerArmManual(double pivotSpeed){
        return new RunCommand(() -> {
            if (!lowerLimitSwitch.get()){
                pivotArm.set(stopSpeed);  // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
            } else {
                pivotArm.set(pivotSpeed);
            }
        }
        // ensures when this command runs, it has sole control of the intake subsystem
        , this
        );
    }
     // Press and hold verision
    public Command raiseArmManual(double pivotSpeed){
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()){ 
                pivotArm.set(stopSpeed); // CONFIRM ROTATION DIRECTION BEFORE RUNNING THIS CODE
            } else {
                pivotArm.set(-pivotSpeed);
            }
        }
        , this
        );
    }
    

    public Command toggleArm(double pivotSpeed){
        return new ConditionalCommand(
            raiseArmAuto(pivotSpeed), // runs when intakeDeployed = true
            lowerArmAuto(pivotSpeed), // runs when intakeDeployed = false
            () -> intakeDeployed // the condition used to determine what command to run
        );
    }
    
    public Command lowerArmAuto(double pivotSpeed){
        return new RunCommand(()->{
            pivotArm.set(pivotSpeed);
        }
        , this)

        .until(()-> !lowerLimitSwitch.get()) //  runs until limit switch is triggered
        .finallyDo(interrupted -> {  // interrupted checks if another commmand has "interrupted" this command/taken control of subsystem. 
            pivotArm.set(stopSpeed);  // So if its true, it will stop motor. If its false, the status wont change, but will still stop the motor.
            if (!interrupted) {
                intakeDeployed = true;
            }
        });
    }

    public Command raiseArmAuto(double pivotSpeed){
        return new RunCommand(()->{
            pivotArm.set(-pivotSpeed);
        }
        , this)
        .until(()-> !upperLimitSwitch.get())
        .finallyDo(interrupted -> { 
            pivotArm.set(stopSpeed);
            if (!interrupted) {
                intakeDeployed = false;
            }
        });
    }


    public Command changeDeployState(){
        return new InstantCommand(()->{
            intakeDeployed = !intakeDeployed;
        });
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivot Arm", pivotArm.getAppliedOutput());
    }
}