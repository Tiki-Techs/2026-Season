package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

/** Controls the intake rollers for collecting game pieces. */
public class Intake extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

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
        SmartDashboard.putNumber("Intake/MotorSpeed", intakeMotor.get());
        SmartDashboard.putNumber("Intake/MotorCurrent", intakeMotor.getOutputCurrent());
    }
}
