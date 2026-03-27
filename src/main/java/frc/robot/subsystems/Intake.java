package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Controls the intake rollers for collecting game pieces. */
public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR, "CANivore");

    public Intake() {
    }

    /** Runs the intake at a specified speed (-1.0 to 1.0). */
    public Command runIntake(double speed) {
        return new RunCommand(() -> intakeMotor.set(speed), this);
    }

    /** Continuously stops the intake motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> intakeMotor.set(0), this);
    }

    /** Stops the intake motor. */
    public Command stopIntake() {
        return new RunCommand(() -> intakeMotor.set(0), this);
    }

    @Override
    public void periodic() {
    }
}
