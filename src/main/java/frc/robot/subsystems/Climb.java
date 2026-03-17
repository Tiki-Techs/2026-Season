package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;

/** Controls the climbing mechanism with limit switch protection. */
public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR, "CANivore");
    private final DigitalInput upperLimitSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH);
    private boolean isCalibrated = false;

    /** Reference to pivot for safety interlock - climb cannot go down while pivot is up. */
    private Pivot pivot = null;

    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(config);
    }

    /** Sets the pivot reference for safety interlock. Must be called after construction. */
    public void setPivot(Pivot pivot) {
        this.pivot = pivot;
    }

    /** Continuously stops the climb motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> climbMotor.set(0), this);
    }

    /** Calibrates the climb by finding the lower limit, then raising to the top. */
    public Command calibrateClimb() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {}, this),

            // Move down until lower limit switch is triggered (bypasses pivot interlock)
            new RunCommand(() -> {
                if (!lowerLimitSwitch.get()) {
                    climbMotor.set(0);
                    return;
                }
                climbMotor.set(-0.3);
            }, this).until(() -> !lowerLimitSwitch.get()),

            // Zero encoder at bottom
            new InstantCommand(() -> {
                climbMotor.set(0.0);
                climbMotor.setPosition(0.0);
                isCalibrated = true;
            }, this),

            // Raise back up to top
            new RunCommand(() -> {
                if (!upperLimitSwitch.get()) {
                    climbMotor.set(0);
                } else {
                    climbMotor.set(0.75);
                }
            }, this).until(() -> !upperLimitSwitch.get()),

            new InstantCommand(() -> climbMotor.set(0.0), this)
        );
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public boolean isUpperLimitPressed() {
        return !upperLimitSwitch.get();
    }

    /** Returns true if the climb is at the lower limit (down position). */
    public boolean isClimbDown() {
        return !lowerLimitSwitch.get();
    }

    /** Runs the climb motor up. Stops at upper limit switch. */
    public Command runClimbUp() {
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(0.75);
            }
        }, this);
    }

    /** Runs the climb motor down. Includes safety interlock with pivot. */
    public Command runClimbDown() {
        return new RunCommand(() -> {
            // Safety: prevent going down while pivot is up
            if (pivot != null && !pivot.isPivotDown()) {
                climbMotor.set(0);
                return;
            }
            if (!lowerLimitSwitch.get()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(-0.5);
            }
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb/IsCalibrated", isCalibrated);
        SmartDashboard.putBoolean("Climb/UpperLimitPressed", isUpperLimitPressed());
        SmartDashboard.putBoolean("Climb/ClimbDown", isClimbDown());
        SmartDashboard.putNumber("Climb/Position", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climb/MotorCurrent", climbMotor.getStatorCurrent().getValueAsDouble());
    }
}
