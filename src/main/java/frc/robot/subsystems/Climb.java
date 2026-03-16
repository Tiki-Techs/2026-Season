package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR);

    // private final DigitalInput upperLimitSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH);
    // private final DigitalInput lowerLimitSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH);

    private boolean isCalibrated = false;

    /** Reference to pivot for safety interlock - climb cannot go down while pivot is up */
    private Pivot pivot = null;

    public Climb() {

    }

    /**
     * Sets the pivot reference for safety interlock.
     * MUST be called after construction for safety to work.
     */
    public void setPivot(Pivot pivot) {
        this.pivot = pivot;
    } 

    public void runClimb(double speed) {
        // SAFETY: Prevent climb from going DOWN (negative speed) while pivot is UP
        // This prevents the climb mechanism from colliding with the pivot arm
        if (speed < 0 && pivot != null && !pivot.isPivotDown()) {
            climbMotor.set(0);
            return;
        }

        // SAFETY: Stop if stalling (hit a hard stop)
        if (climbMotor.getStatorCurrent().getValueAsDouble() > ClimbConstants.HOMING_STALL_AMPS) {
            climbMotor.set(0);
            return;
        }

        climbMotor.set(speed);
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

    public Command calibrateClimb() {
        return new RunCommand(() -> {
            climbMotor.set(0.15);
        }, this)
            .until(() -> (climbMotor.getStatorCurrent().getValueAsDouble() > ClimbConstants.HOMING_STALL_AMPS))
            .andThen(new InstantCommand(() -> {
                climbMotor.set(0.0);
                climbMotor.setPosition(0.0);
                isCalibrated = true;
            }));
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    /**
     * Runs the climb motor up (positive direction) at a fast speed.
     * Used for safety interlock when pivot is raising.
     * Stops if stall is detected.
     */
    public Command runClimbUp() {
        return new RunCommand(() -> {
            // Stop if stalling (hit upper limit)
            if (climbMotor.getStatorCurrent().getValueAsDouble() > ClimbConstants.HOMING_STALL_AMPS) {
                climbMotor.set(0);
            } else {
                climbMotor.set(0.5);
            }
        }, this);
    }

    @Override
    public void periodic() {
        double current = climbMotor.getStatorCurrent().getValueAsDouble();
        SmartDashboard.putNumber("Climb/StallCurrent", current);
        SmartDashboard.putBoolean("Climb/IsCalibrated", isCalibrated);
        // Safety status - shows if climb down is allowed (pivot must be down)
        SmartDashboard.putBoolean("Climb/DownAllowed", pivot == null || pivot.isPivotDown());
        // Shows if climb is currently stalling (at a hard stop)
        SmartDashboard.putBoolean("Climb/IsStalling", current > ClimbConstants.HOMING_STALL_AMPS);
    }
    
}
