package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Controls the intake rollers for collecting game pieces. */
public class Intake extends SubsystemBase {

    private final TalonFX intakeLeft = new TalonFX(IntakeConstants.INTAKE_LEFT_MOTOR, "CANivore");
    private final TalonFX intakeRight = new TalonFX(IntakeConstants.INTAKE_RIGHT_MOTOR, "CANivore");

    public Intake() {
        // Right opposes left so both rollers spin inward
        intakeRight.setControl(new Follower(IntakeConstants.INTAKE_LEFT_MOTOR, MotorAlignmentValue.Opposed));
    }

    /** Runs the intake at a specified speed (-1.0 to 1.0). */
    public Command runIntake(double speed) {
        return new RunCommand(() -> intakeLeft.set(-speed), this);
    }

    /** Continuously stops the intake motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> intakeLeft.set(0), this);
    }

    /** Stops the intake motor. */
    public Command stopIntake() {
        return new RunCommand(() -> intakeLeft.set(0), this);
    }

    @Override
    public void periodic() {
    }
}
