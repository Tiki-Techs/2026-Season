package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.autoclimb.AutoclimbConstants;

/**
 * Controls the climb mechanism (TalonFX CAN 31, CANivore bus).
 *
 * Encoder convention (set by calibrateClimb):
 *   0.0            = upper hard stop (arm fully retracted)
 *   negative value = arm extended downward (more negative = further down)
 *
 * IMPORTANT: calibrateClimb() MUST complete successfully before any
 * position-based command (raiseToRungHeight, lift, stow) is called.
 * Soft limits are NOT applied at construction — only after calibration.
 */
public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR, "CANivore");
    private final DigitalInput upperLimitSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH);

    private boolean isCalibrated = false;
    private double measuredLowerRotations = AutoclimbConstants.SOFT_LIMIT_REVERSE_ROTATIONS;

    private final PositionVoltage m_posRequest = new PositionVoltage(0).withSlot(0);

    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);

        // Slot 0 — PositionVoltage gains. Tune CLIMBER_KP with Phoenix Tuner X on robot.
        config.Slot0 = new Slot0Configs()
            .withKP(AutoclimbConstants.CLIMBER_KP);

        // NOTE: Soft limits are NOT enabled here. They are applied only after calibration
        // completes (see calibrateClimb). Enabling them in the constructor with encoder=0
        // would block the upward calibration move immediately.

        climbMotor.getConfigurator().apply(config);
    }

    // =========================================================================
    // CALIBRATION
    // =========================================================================

    /**
     * Finds the upper limit switch, zeros the encoder, then descends to the lower limit.
     *
     * After this command finishes:
     *   - encoder position = 0 at upper hard stop
     *   - soft limits are enabled at [+0.5, SOFT_LIMIT_REVERSE_ROTATIONS]
     *   - isCalibrated() returns true
     *
     * The command completes when the lower limit switch is triggered.
     */
    public Command calibrateClimb() {
        return new SequentialCommandGroup(

            // Clear calibration flag and disable soft limits
            new InstantCommand(() -> {
                isCalibrated = false;
                applySoftLimits(false);
            }, this),

            // Drive DOWN to lower limit switch (single trip — limit switches replace encoder homing)
            new RunCommand(() -> climbMotor.set(-ClimbConstants.CALIB_SPEED_DOWN), this)
                .until(this::isLowerLimitPressed)
                .withTimeout(15.0),

            // Zero encoder at lower position, mark calibrated
            new InstantCommand(() -> {
                climbMotor.set(0);
                climbMotor.setPosition(0.0);
                measuredLowerRotations = 0.0;
                isCalibrated = true;
            }, this)
        );
    }

    // =========================================================================
    // POSITION COMMANDS (require isCalibrated == true)
    // =========================================================================

    /**
     * Drives the arm upward until the upper limit switch is hit.
     * Used in the LIFTING phase of autoclimb to raise to max height before pull-down.
     * Runs until interrupted.
     */
    public Command raiseToRungHeight() {
        return new RunCommand(() -> {
            if (isUpperLimitPressed()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1.0);
            }
        }, this);
    }

    /**
     * Drives the arm to CLIMBER_LIFT_ROTATIONS (fully retracted, lifts the robot).
     * Runs until interrupted.
     */
    public Command lift() {
        if (!isCalibrated) return warnNotCalibrated("lift");
        return new RunCommand(() ->
            climbMotor.setControl(m_posRequest
                .withPosition(AutoclimbConstants.CLIMBER_LIFT_ROTATIONS)),
            this
        );
    }

    /**
     * Drives the arm to the stow position near the upper hard stop.
     * Completes when within tolerance of the stow position.
     */
    public Command stow() {
        if (!isCalibrated) return warnNotCalibrated("stow");
        return new RunCommand(() ->
            climbMotor.setControl(m_posRequest
                .withPosition(AutoclimbConstants.CLIMBER_STOW_ROTATIONS)),
            this
        ).until(() -> isNearPosition(AutoclimbConstants.CLIMBER_STOW_ROTATIONS));
    }

    // =========================================================================
    // MANUAL COMMANDS
    // =========================================================================

    /** Runs climb motor upward. Stops at upper limit switch. */
    public Command runClimbUp() {
        return new RunCommand(() -> {
            if (isUpperLimitPressed()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1.0);
            }
        }, this);
    }

    /** Runs climb motor downward. Stops at lower limit switch. */
    public Command runClimbDown() {
        return new RunCommand(() -> {
            if (isLowerLimitPressed()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(-1.0);
            }
        }, this);
    }

    /**
     * Lowers the arm at reduced speed for the auto-climb engagement phase.
     * Slow speed lets the O settle gently onto the rung before bearing robot weight.
     * Stops at lower limit switch (or stalls naturally when rung is loaded inside the O).
     * The TalonFX 40A current limit protects the motor if the rung blocks full travel.
     */
    public Command lowerForClimb() {
        return new RunCommand(() -> {
            if (isLowerLimitPressed()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(-AutoclimbConstants.LOWER_CLIMB_SPEED);
            }
        }, this);
    }

    /**
     * Fast duty-cycle retract toward upper limit switch — for abort/emergency.
     * Does not use PositionVoltage. Stops at upper limit.
     */
    public Command emergencyRetract() {
        return new RunCommand(() -> {
            if (isUpperLimitPressed()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1.0);
            }
        }, this).until(this::isUpperLimitPressed);
    }

    /** Continuously stops the climb motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> climbMotor.set(0), this);
    }

    /**
     * Raises the motor directly — no subsystem requirement declared.
     * For use by AutoclimbCommand inline only (avoids scheduler conflict with stopAll default).
     */
    public void raiseMotor() {
        if (isUpperLimitPressed()) {
            climbMotor.set(0);
        } else {
            climbMotor.set(1.0);
        }
    }

    /**
     * Lowers the motor slowly — no subsystem requirement declared.
     * For use by AutoclimbCommand inline only (avoids scheduler conflict with stopAll default).
     */
    public void lowerMotorSlow() {
        if (isLowerLimitPressed()) {
            climbMotor.set(0);
        } else {
            climbMotor.set(-AutoclimbConstants.LOWER_CLIMB_SPEED);
        }
    }

    // =========================================================================
    // STATE ACCESSORS
    // =========================================================================

    public boolean isCalibrated()       { return isCalibrated; }
    public boolean isUpperLimitPressed(){ return !upperLimitSwitch.get(); }
    public boolean isLowerLimitPressed(){ return !lowerLimitSwitch.get(); }
    public double  getPosition()        { return climbMotor.getPosition().getValueAsDouble(); }
    public double  getStatorCurrent()   { return climbMotor.getStatorCurrent().getValueAsDouble(); }

    public boolean isAtRungHeight() {
        return isUpperLimitPressed();
    }

    // =========================================================================
    // PRIVATE HELPERS
    // =========================================================================

    private boolean isNearPosition(double targetRotations) {
        return Math.abs(getPosition() - targetRotations)
            < AutoclimbConstants.CLIMBER_POSITION_TOLERANCE_ROTATIONS;
    }

    private void applySoftLimits(boolean enabled) {
        var cfg = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(AutoclimbConstants.SOFT_LIMIT_FORWARD_ROTATIONS)
            .withForwardSoftLimitEnable(enabled)
            .withReverseSoftLimitThreshold(measuredLowerRotations)
            .withReverseSoftLimitEnable(enabled);
        climbMotor.getConfigurator().apply(cfg);
    }

    /** Returns a no-op command and logs a warning when called before calibration. */
    private Command warnNotCalibrated(String methodName) {
        return new InstantCommand(() ->
            System.err.println("[Climb] WARNING: " + methodName +
                " called before calibration. Run calibrateClimb() first.")
        , this);
    }

    // =========================================================================
    // PERIODIC
    // =========================================================================

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb/IsCalibrated",          isCalibrated);
        SmartDashboard.putBoolean("Climb/UpperLimit",             isUpperLimitPressed());
        SmartDashboard.putBoolean("Climb/LowerLimit",             isLowerLimitPressed());
        SmartDashboard.putNumber("Climb/Position",                getPosition());
        SmartDashboard.putNumber("Climb/StatorCurrent",           getStatorCurrent());
        SmartDashboard.putNumber("Climb/MeasuredLowerRotations",  measuredLowerRotations);
    }
}
