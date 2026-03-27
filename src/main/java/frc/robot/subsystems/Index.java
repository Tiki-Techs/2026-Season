package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexConstants;

/** Transfers game pieces from the intake to the shooter using belt-driven rollers. */
public class Index extends SubsystemBase {

    private final TalonFX indexMotor = new TalonFX(IndexConstants.INDEX_MOTOR, "CANivore");

    public Index() {
        var currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);
        indexMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentLimits));
    }

    /** Runs the indexer at a fixed speed (-1.0 to 1.0). */
    public Command runIndex(double speed) {
        return new RunCommand(() -> indexMotor.set(speed), this);
    }

    /** Immediately stops the indexer motor. */
    public Command stopIndex() {
        return new InstantCommand(() -> indexMotor.set(0), this);
    }

    /** Continuously stops the motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> indexMotor.set(0), this);
    }

    @Override
    public void periodic() {
    }
}
