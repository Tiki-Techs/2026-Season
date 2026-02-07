package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeActuator extends SubsystemBase{


    // private final SparkMax leaderIntake = new SparkMax(24, MotorType.kBrushless);
    // private final SparkMax followerIntake = new SparkMax(25, MotorType.kBrushless);

    private final SparkMax pivotArm = new SparkMax(28, MotorType.kBrushless); // CanSpark Max with Neo brushless motor

    // when requesting a digital input, the boolean value will always be true if it is unplugged. 
    private final DigitalInput lowerLimitSwitch = new DigitalInput(3);
    private final DigitalInput upperLimitSwitch = new DigitalInput(2);

    private final double pivotSpeed = 0.1;
    private final double stopSpeed = 0.0;

    private boolean intakeDeployed = true; 
    
     public Command stopAll(){
        return new RunCommand(()->{
            pivotArm.set(stopSpeed);
        },
        this
        );
    }

    // Press and hold verision
    public Command lowerArmManual(){
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
    public Command raiseArmManual(){
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
// // test code for auto deploy
//     private boolean intakeDeployed = true;
    // public Command toggleIntake(){
    //     return new InstantCommand(()->{
    //         if(intakeDeployed){
    //             raiseIntakeAuto()
    //             .schedule();
    //         } else {
    //             lowerIntakeAuto()
    //             .schedule();
    //         }
    //         intakeDeployed = !intakeDeployed;
    //     }
    //     // , this 
    //     );
    // }

    public Command toggleArm(){
        return new ConditionalCommand(
            raiseArmAuto(), // runs when intakeDeployed = true
            lowerArmAuto(), // runs when intakeDeployed = false
            () -> intakeDeployed // the condition used to determine what command to run
        );
    }
    
    public Command lowerArmAuto(){
        return new RunCommand(()->{
            pivotArm.set(pivotSpeed);
        }
        , this)
        .until(()-> !lowerLimitSwitch.get()) // runs until limit switch is triggered
        .unless(()-> !lowerLimitSwitch.get()) // prevents running if limit switch is already triggered
        .finallyDo(interrupted -> { 
            pivotArm.set(stopSpeed); 
            if (!interrupted) {
                intakeDeployed = true;
            }
        });
    }

    public Command raiseArmAuto(){
        return new RunCommand(()->{
            pivotArm.set(-pivotSpeed);
        }
        , this)
        .until(()-> !upperLimitSwitch.get())
        .unless(()-> !upperLimitSwitch.get())
        .finallyDo(interrupted -> {
            pivotArm.set(stopSpeed);
            if (!interrupted) {
                intakeDeployed = false;
            }
        });
    }




    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("lower limit", lowerLimitSwitch.get());
        SmartDashboard.putBoolean("upper limit", upperLimitSwitch.get());

        SmartDashboard.putData("Auto lower intake command", lowerArmAuto());
        SmartDashboard.putData("Toggle intake test command", raiseArmAuto());
    }
}