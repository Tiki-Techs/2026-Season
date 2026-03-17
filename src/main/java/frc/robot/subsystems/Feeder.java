package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/** Controls the feeder roller that feeds game pieces to the shooter. */
public class Feeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(FeederConstants.FEEDER, "CANivore");

    /** Runs the feeder at a specified speed (-1.0 to 1.0). */
    public Command runFeeder(double speed) {
        return new RunCommand(() -> feederMotor.set(speed), this);
    }

    /** Stops the feeder motor. */
    public Command stopFeeder() {
        return new RunCommand(() -> feederMotor.set(0), this);
    }

    /** Continuously stops the feeder motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> feederMotor.set(0), this);
    }
}
