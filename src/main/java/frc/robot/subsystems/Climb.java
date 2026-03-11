package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR);

    private final DigitalInput upperLimitSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH);

    public Climb() {

    } 

    public void runClimb(double speed) {
        // Stop if trying to go down and at lower limit, or trying to go up and at upper limit
        if ((speed > 0 && !lowerLimitSwitch.get()) || (speed < 0 && !upperLimitSwitch.get())) {
            climbMotor.set(0);
        } else {
            climbMotor.set(speed);
        }
    }

    public Command runClimbCommand(DoubleSupplier speedSupplier) {
        return new RunCommand(() -> runClimb(speedSupplier.getAsDouble()), this);
    }

    public Command stopAll() {
        return new RunCommand(() ->
            climbMotor.set(0),
            this
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb/UpperLimit", !upperLimitSwitch.get());
        SmartDashboard.putBoolean("Climb/LowerLimit", !lowerLimitSwitch.get());
    }
    
}
