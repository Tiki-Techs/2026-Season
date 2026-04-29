package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

/**
 * Controls the intake arm pivot mechanism.
 *
 * Hardware: SparkMax (CAN 15) + NEO brushless. No limit switches.
 *
 * Calibration (current-spike hard stop detection):
 *   1. Drive DOWN slowly until motor stalls against lower hard stop → zero encoder (lowerEncoderPos = 0).
 *   2. Drive UP slowly until motor stalls against upper hard stop → record upperEncoderPos (negative).
 *
 * All movement commands use encoder soft limits computed from calibration.
 */
public class Pivot extends SubsystemBase {

    private final SparkMax pivotArm = new SparkMax(PivotConstants.PIVOT_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = pivotArm.getEncoder();

    private boolean isCalibrated = false;
    private double lowerEncoderPos = 0.0;   // Encoder position at lower physical stop (set to 0 after calibration)
    private double upperEncoderPos = -7.5;  // Encoder position at upper physical stop (measured by calibration)

    // Timer used inside calibratePivot() to confirm stall duration
    private final Timer stallTimer = new Timer();

    public Pivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.inverted(true); // Motor is physically wired backwards; invert so set(negative) = raise
        // Smart current limit prevents motor from burning out during normal operation.
        // Stall detection uses a lower software threshold so we can stop before the limit engages.
        config.smartCurrentLimit(40);
        pivotArm.configure(config,
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
            com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
    }

    // =========================================================================
    // CALIBRATION
    // =========================================================================

    /**
     * Finds both physical hard stops using current-spike stall detection.
     *
     * Step 1 — lower stop: drive down slowly; when output current exceeds
     *   STALL_CURRENT_AMPS for STALL_DURATION_S, motor is stalled → zero encoder.
     * Step 2 — upper stop: drive up slowly; same stall detection → record upperEncoderPos.
     *
     * After this command completes, isCalibrated() returns true and all movement
     * commands have valid soft limits.
     */
    public Command calibratePivot() {
        return new SequentialCommandGroup(

            // Reset state
            new InstantCommand(() -> {
                isCalibrated = false;
                stallTimer.reset();
                stallTimer.stop();
            }, this),

            // Step 1: drive DOWN until stall
            new RunCommand(() -> {
                double current = pivotArm.getOutputCurrent();
                if (current > PivotConstants.STALL_CURRENT_AMPS) {
                    stallTimer.start();
                } else {
                    stallTimer.reset();
                    stallTimer.stop();
                }
                pivotArm.set(PivotConstants.AUTO_PIVOT_SPEED);
            }, this).until(() -> stallTimer.hasElapsed(PivotConstants.STALL_DURATION_S)),

            // Zero encoder at lower hard stop
            new InstantCommand(() -> {
                pivotArm.set(0);
                encoder.setPosition(0.0);
                lowerEncoderPos = 0.0;
                stallTimer.reset();
                stallTimer.stop();
            }, this),

            // Brief pause before driving the other way
            new RunCommand(() -> pivotArm.set(0), this)
                .withTimeout(0.1),

            // Step 2: drive UP until stall
            new RunCommand(() -> {
                double current = pivotArm.getOutputCurrent();
                if (current > PivotConstants.STALL_CURRENT_AMPS) {
                    stallTimer.start();
                } else {
                    stallTimer.reset();
                    stallTimer.stop();
                }
                pivotArm.set(-PivotConstants.AUTO_PIVOT_SPEED);
            }, this).until(() -> stallTimer.hasElapsed(PivotConstants.STALL_DURATION_S)),

            // Record upper encoder position and mark calibrated
            new InstantCommand(() -> {
                pivotArm.set(0);
                upperEncoderPos = encoder.getPosition();
                isCalibrated = true;
                stallTimer.reset();
                stallTimer.stop();
            }, this)
        );
    }

    // =========================================================================
    // MOVEMENT COMMANDS
    // =========================================================================

    /**
     * Raises the arm (toward upper hard stop).
     * Slows near the upper limit. Stops at upper limit.
     *
     * @param speed Full-speed duty cycle (0–1.0). Use PivotConstants.RAISE_SPEED.
     */
    public Command raiseArmManual(double speed) {
        return new RunCommand(() -> {
            double pos = encoder.getPosition();
            double distToUpper = Math.abs(pos - upperEncoderPos);
            double totalTravel = Math.abs(upperEncoderPos - lowerEncoderPos);
            double slowZone = totalTravel * PivotConstants.SLOW_ZONE_FRACTION;

            if (isCalibrated && distToUpper < 0.05) {
                pivotArm.set(0);
            } else if (isCalibrated && distToUpper < slowZone) {
                pivotArm.set(-PivotConstants.SLOW_ZONE_SPEED);
            } else {
                pivotArm.set(-speed);
            }
        }, this);
    }

    /**
     * Lowers the arm (toward lower hard stop).
     * Slows near the lower limit. Stops at lower limit.
     *
     * @param speed Full-speed duty cycle (0–1.0). Use PivotConstants.LOWER_SPEED.
     */
    public Command lowerArmManual(double speed) {
        return new RunCommand(() -> {
            double pos = encoder.getPosition();
            double distToLower = Math.abs(pos - lowerEncoderPos);
            double totalTravel = Math.abs(upperEncoderPos - lowerEncoderPos);
            double slowZone = totalTravel * PivotConstants.SLOW_ZONE_FRACTION;

            if (isCalibrated && distToLower < 0.05) {
                pivotArm.set(0);
            } else if (isCalibrated && distToLower < slowZone) {
                pivotArm.set(PivotConstants.SLOW_ZONE_SPEED);
            } else {
                pivotArm.set(speed);
            }
        }, this);
    }

    /**
     * Drives pivot to the upper (stowed) position and holds there.
     * Used by the dump sequence. Runs until interrupted.
     * Safe to call before calibration — will just stop if not calibrated.
     */
    public Command raiseToTop() {
        return new RunCommand(() -> {
            if (!isCalibrated) {
                pivotArm.set(0);
                return;
            }
            double pos = encoder.getPosition();
            double distToUpper = Math.abs(pos - upperEncoderPos);
            double totalTravel = Math.abs(upperEncoderPos - lowerEncoderPos);
            double slowZone = totalTravel * PivotConstants.SLOW_ZONE_FRACTION;

            if (distToUpper < 0.05) {
                pivotArm.set(0);
            } else if (distToUpper < slowZone) {
                pivotArm.set(-PivotConstants.SLOW_ZONE_SPEED);
            } else {
                pivotArm.set(-PivotConstants.AUTO_PIVOT_SPEED);
            }
        }, this);
    }

    /**
     * Simple open-loop pivot control with both-direction soft stops (when calibrated).
     * Positive speed = lower, negative = raise.
     */
    public Command runPivot(double pivotSpeed) {
        return new RunCommand(() -> {
            if (!isCalibrated) {
                pivotArm.set(pivotSpeed);
                return;
            }
            double pos = encoder.getPosition();
            double totalTravel = Math.abs(upperEncoderPos - lowerEncoderPos);
            double slowZone = totalTravel * PivotConstants.SLOW_ZONE_FRACTION;

            if (pivotSpeed > 0) {
                // Moving toward lower hard stop
                double distToLower = Math.abs(pos - lowerEncoderPos);
                if (distToLower < 0.05) {
                    pivotArm.set(0);
                } else if (distToLower < slowZone) {
                    pivotArm.set(PivotConstants.SLOW_ZONE_SPEED);
                } else {
                    pivotArm.set(pivotSpeed);
                }
            } else if (pivotSpeed < 0) {
                // Moving toward upper hard stop
                double distToUpper = Math.abs(pos - upperEncoderPos);
                if (distToUpper < 0.05) {
                    pivotArm.set(0);
                } else if (distToUpper < slowZone) {
                    pivotArm.set(-PivotConstants.SLOW_ZONE_SPEED);
                } else {
                    pivotArm.set(pivotSpeed);
                }
            } else {
                pivotArm.set(0);
            }
        }, this);
    }

    /** Continuously stops the pivot motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> pivotArm.set(0), this);
    }

    // =========================================================================
    // STATE ACCESSORS
    // =========================================================================

    public boolean isCalibrated() { return isCalibrated; }

    /** Returns true when the arm is within 0.05 encoder units of the lower (deployed) hard stop. */
    public boolean isAtLowerLimit() {
        return isCalibrated && Math.abs(encoder.getPosition() - lowerEncoderPos) < 0.05;
    }

    /** Returns true when the arm is within 0.05 encoder units of the upper (stowed) hard stop. */
    public boolean isAtUpperLimit() {
        return isCalibrated && Math.abs(encoder.getPosition() - upperEncoderPos) < 0.05;
    }

    public double getEncoderPosition() { return encoder.getPosition(); }

    // =========================================================================
    // PERIODIC
    // =========================================================================

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pivot/IsCalibrated", isCalibrated);
        SmartDashboard.putNumber("Pivot/EncoderPosition", encoder.getPosition());
        SmartDashboard.putNumber("Pivot/OutputCurrent", pivotArm.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/LowerEncoderPos", lowerEncoderPos);
        SmartDashboard.putNumber("Pivot/UpperEncoderPos", upperEncoderPos);
    }
}
