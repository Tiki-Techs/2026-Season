package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.climbMotor);

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);


    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(config);
        climbMotor.setPosition(0);
    }

    // public Command climbUp(DoubleSupplier speed) {
    //     return new RunCommand(() ->
    //         climbMotor.setControl(dutyCycleRequest.withOutput(speed)), this);
    // }

    // public Command climbDown(DoubleSupplier speed) {
    //     return new RunCommand(() ->
    //         climbMotor.setControl(dutyCycleRequest.withOutput(speed)), this);
    // }

    public Command stopAll() {
        return new RunCommand(() ->
            climbMotor.setControl(dutyCycleRequest.withOutput(0)), this);
    }

    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Position", getPosition());
    }
}
