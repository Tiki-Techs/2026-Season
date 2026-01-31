package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 


public class Shooter extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(20);
    private final TalonFX shooterMotor = new TalonFX(21);
    private final BangBangController bangBang = new BangBangController();

    private static final double SHOOTER_TARGET_RPS = 80.0; // Target rotations per second
    private static final double SHOOTER_TOLERANCE_RPS = 2.0; // Tolerance for "at speed"

    public Command runShooterIntake(CommandXboxController controllerValue){
        return new RunCommand(() -> 
            intakeMotor.set(controllerValue.getRightTriggerAxis())
    );}

    public Command runShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> 
            shooterMotor.set(controllerValue.getLeftTriggerAxis())
    );}

    public Command stopAll(){
        return new RunCommand(()->{
                shooterMotor.set(0);
                intakeMotor.set(0);
            },
            this
        );
    }

    public Command spinUpShooter() {
        return new RunCommand(() -> {
            double currentVelocity = shooterMotor.getVelocity().getValueAsDouble();
            double output = bangBang.calculate(currentVelocity, SHOOTER_TARGET_RPS);
            shooterMotor.set(output);
        }, this);
    }

    public boolean isShooterAtSpeed() {
        double currentVelocity = shooterMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - SHOOTER_TARGET_RPS) < SHOOTER_TOLERANCE_RPS;
    }
}