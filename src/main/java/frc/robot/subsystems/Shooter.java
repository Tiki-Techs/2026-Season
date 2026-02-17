package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * Shooter subsystem that controls the flywheel shooter mechanism.
 * Uses two TalonFX (Kraken) motors to launch game pieces.
 * Supports both open-loop (manual) and closed-loop (PID) velocity control.
 */
public class Shooter extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Primary shooter motor (CAN ID 21) */
    private final TalonFX shooter21 = new TalonFX(21);

    /** Secondary shooter motor (CAN ID 22) */
    private final TalonFX shooter22 = new TalonFX(22);

    // ==================== CONTROL PARAMETERS ====================

    /** Target velocity for bang-bang control mode (rotations per second) */
    private double shooterTargetVelocity = 0;

    /** Power applied when shooter is below setpoint in bang-bang control */
    private final double BANG_BANG_ON_POWER = 1.0;

    /** Power applied when shooter reaches/exceeds setpoint in bang-bang control */
    private final double BANG_BANG_OFF_POWER = 0.00;

    /** Velocity voltage control request for closed-loop PID control */
    final VelocityVoltage shooterVoltageRequest = new VelocityVoltage(0.0).withSlot(0);

    /**
     * Constructs the Shooter subsystem and configures motor PID gains.
     * Sets up Slot 0 with feedforward and PID constants for velocity control.
     */
    public Shooter() {
        // Configure PID gains for velocity control (Slot 0)
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1;  // Static friction compensation (volts)
        slot0Configs.kV = 0.12; // Velocity feedforward (volts per RPS)
        slot0Configs.kP = 0.11; // Proportional gain (volts per RPS of error)
        slot0Configs.kI = 0;    // Integral gain (disabled)
        slot0Configs.kD = 0;    // Derivative gain (disabled)

        // Apply PID configuration to both shooter motors
        shooter21.getConfigurator().apply(slot0Configs);
        shooter22.getConfigurator().apply(slot0Configs);
    }

    // ==================== PID VELOCITY CONTROL ====================

    /**
     * Runs the shooter at a target velocity using closed-loop PID control.
     * Uses velocity voltage control with feedforward for accurate speed control.
     *
     * @param targetRPS Target velocity in rotations per second
     * @return Command that continuously runs the shooter at the target velocity
     */
    public Command runPIDShooter(double targetRPS) {
        return new RunCommand(() -> {
            // Apply velocity control with 0.5V feedforward to overcome friction/gravity
            shooter21.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
            shooter22.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
        }, this);
    }

    // ==================== OPEN-LOOP CONTROL ====================

    /**
     * Runs the shooter using controller trigger input for variable speed.
     * Uses open-loop duty cycle control (no velocity feedback).
     *
     * @param controllerValue Xbox controller to read trigger axis from
     * @return Command that sets shooter speed based on left trigger position
     */
    public Command runShooter(CommandXboxController controllerValue) {
        return new RunCommand(() -> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooter21.set(-speed);
            shooter22.set(speed);
        }, this);
    }

    /**
     * Runs the shooter at a fixed speed using open-loop control.
     *
     * @param speed Motor output from -1.0 to 1.0
     * @return Command that continuously runs the shooter at the specified speed
     */
    public Command runShooter(double speed) {
        return new RunCommand(() -> {
            shooter21.set(-speed);
            shooter22.set(-speed);
        }, this);
    }

    /**
     * Runs the shooter in reverse using controller trigger input.
     * Useful for unjamming or ejecting game pieces.
     *
     * @param controllerValue Xbox controller to read trigger axis from
     * @return Command that sets reverse shooter speed based on left trigger
     */
    public Command runReverseShooter(CommandXboxController controllerValue) {
        return new RunCommand(() -> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooter21.set(speed);
            shooter22.set(speed);
        }, this);
    }

    // ==================== STOP COMMANDS ====================

    /**
     * Immediately stops the shooter motors (instant command).
     * Executes once and completes.
     *
     * @return InstantCommand that sets both motors to zero
     */
    public Command stopShooter() {
        return new InstantCommand(() -> {
            shooter21.set(0);
            shooter22.set(0);
        }, this);
    }

    /**
     * Continuously commands the shooter motors to stop.
     * Use as a default command to ensure motors stay stopped when not in use.
     *
     * @return RunCommand that continuously sets both motors to zero
     */
    public Command stopAll() {
        return new RunCommand(() -> {
            shooter21.set(0);
            shooter22.set(0);
        }, this);
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Sets the target velocity for bang-bang control mode.
     *
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setShooterTargetVelocity(double velocityRPS) {
        this.shooterTargetVelocity = velocityRPS;
    }

    /**
     * Gets the current shooter flywheel velocity.
     *
     * @return Current velocity in rotations per second
     */
    public double getShooterVelocity() {
        return shooter21.getVelocity().getValueAsDouble();
    }

    /**
     * Checks if the shooter has reached the target velocity within tolerance.
     * Useful for determining when the shooter is ready to fire.
     *
     * @param tolerance Acceptable error in rotations per second
     * @return True if current velocity is within tolerance of target
     */
    public boolean isAtTargetVelocity(double tolerance) {
        return Math.abs(getShooterVelocity() - shooterTargetVelocity) <= tolerance;
    }

    @Override
    public void periodic() {
        // Periodic updates (telemetry can be added here)
    }
}
