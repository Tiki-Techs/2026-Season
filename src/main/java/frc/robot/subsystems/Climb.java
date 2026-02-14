// package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climb extends SubsystemBase {

//     private final TalonFX climbMotor = new TalonFX(30);
//     private final DigitalInput lowerLimitSwitch = new DigitalInput(4);
//     private final DigitalInput upperLimitSwitch = new DigitalInput(5);




//     public Climb() {
//         TalonFXConfiguration config = new TalonFXConfiguration();
//         config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         climbMotor.getConfigurator().apply(config);
//         climbMotor.setPosition(0);
        
//     }

//     public Command climbUpManual(double speed) {
//         return new RunCommand(() ->
//             climbMotor.set(speed), this);
//     }

//     public Command climbDownManual(double speed) {
//         return new RunCommand(() ->
//             climbMotor.set(-speed), this);
//     }

//     // public Command climbUpAuto(){
//     //     return new RunCommand(() -> {
//     //         if (!upperLimitSwitch.get()){
//     //             climbMotor.set(0.1);
//     //         } else {
//     //             climbMotor.set(0);
//     //         }

//     //     });
//     // }

//     // public Command climbDownAuto(){
//     //     return new RunCommand(() -> {
//     //         if (!lowerLimitSwitch.get()){
//     //             climbMotor.set(-0.1);
//     //         } else {
//     //             climbMotor.set(0);
//     //         }

//     //     });
//     // }

//     public Command climbDownAuto(){
//         return new RunCommand(() -> {
//             climbMotor.set(-0.1);
//             }, this).until(()->!lowerLimitSwitch.get())
//             .finallyDo(()->climbMotor.set(0));
//     }

//     public Command climbUpAuto(){
//         return new RunCommand(() -> {
//             climbMotor.set(0.1);
//             }, this).until(()->!upperLimitSwitch.get())
//             .finallyDo(()->climbMotor.set(0));
//     }

//     public Command stopAll() {
//         return new RunCommand(() ->
//             climbMotor.set(0), this);
//     }

//     public double getPosition() {
//         return climbMotor.getPosition().getValueAsDouble();
//     }

//     public Command resetPosition() {
//         return new RunCommand(() -> {
//             climbMotor.set(-0.1);
//         }, this).until(()->lowerLimitSwitch.get())
//         .andThen(()->{
//         climbMotor.set(0);
//         climbMotor.setPosition(0);
//         });
//     }


//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Climb/Position", getPosition());
//         SmartDashboard.putBoolean("Climb Lower Limit", lowerLimitSwitch.get());
//         SmartDashboard.putBoolean("Climb Upper Limit", upperLimitSwitch.get());
//     }
// }
