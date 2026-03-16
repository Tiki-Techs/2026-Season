package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.HoodConstants;

/**
 * Hood subsystem that controls the shooter hood angle for trajectory adjustment.
 * Uses a TalonFX motor to adjust the launch angle based on distance to target.
 * Supports automatic aiming using distance (meters) with an interpolating lookup table.
 */
public class Hood extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Hood angle adjustment motor */
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR);


    // ==================== DEPENDENCIES ====================

    /** Vision subsystem for distance-based aiming */
    private final Vision vision;

    // ==================== CONTROL ====================

    /** PID controller for closed-loop position control */
    private final PIDController pidController = new PIDController(0.1, 0, 0);

    /**
     * Lookup table mapping distance (meters) to hood motor positions.
     * Interpolates between data points for smooth hood adjustment.
     * Farther distance = higher hood angle (more negative encoder position).
     */
    private final InterpolatingDoubleTreeMap distanceToHoodPosition = new InterpolatingDoubleTreeMap();

    // ==================== POSITION LIMITS ====================

    /** Minimum hood position (encoder rotations) - fully down */
    private double minPosition = 0.3;

    /** Maximum hood position (encoder rotations) - fully up */
    private double maxPosition = -3.0;

    /** Current target position for PID control */
    private double targetPosition = 0.0;

    private boolean isCalibrated = false;


    /**
     * Constructs the Hood subsystem and initializes configuration.
     * Zeros the motor encoder and populates the distance-to-position lookup table.
     *
     * @param vision Vision subsystem for distance measurements
     */
    public Hood(Vision vision) {
        this.vision = vision;

        // Reset encoder position on startup
        hoodMotor.setPosition(0);

        // Populate lookup table with tuned values
        // Format: distanceToHoodPosition.put(distanceMeters, hoodEncoderPosition)
        // Farther distance = higher hood angle (more negative encoder value)
        // TODO: Calibrate these values by testing at known distances
        distanceToHoodPosition.put(1.0, -0.1);   // Close shot (~1m)
        distanceToHoodPosition.put(2.0, -0.4);   // Mid-close shot (~2m)
        distanceToHoodPosition.put(3.0, -0.7);   // Mid-far shot (~3m)
        distanceToHoodPosition.put(4.0, -1.0);   // Far shot (~4m)

        // Set position tolerance for "at setpoint" checks
        pidController.setTolerance(0.25);
    }

    // ==================== LOOKUP TABLE ====================

    /**
     * Gets the ideal hood position for a given distance in meters.
     * Uses interpolation between defined data points.
     *
     * @param distanceMeters Distance to target in meters
     * @return Hood position in motor rotations, clamped to safe limits
     */
    public double getHoodPositionForDistance(double distanceMeters) {
        double position = distanceToHoodPosition.get(distanceMeters);
        return Math.max(maxPosition, Math.min(minPosition, position));
    }

    // ==================== AUTO-AIM ====================

    /**
     * Automatically adjusts hood angle based on distance to goal.
     * Uses robot odometry position to calculate distance - works even if
     * the Limelight can't see the goal directly.
     *
     * @return Command that continuously auto-aims the hood while running
     */
    public Command autoAimHood() {
        return new RunCommand(() -> {
            // Use odometry-based distance to goal (always available)
            double distance = vision.getDistanceToGoal();
            targetPosition = getHoodPositionForDistance(distance);

            // PID control to reach target position
            double output = pidController.calculate(
                hoodMotor.getPosition().getValueAsDouble(),
                targetPosition
            );
            hoodMotor.set(output);
        }, this);
    }

    // ==================== POSITION CONTROL ====================

    /**
     * Moves the hood to a specific encoder position using PID control.
     * Position is clamped to safe limits.
     *
     * @param position Target position in motor rotations
     * @return Command that moves hood to the specified position
     */
    public Command setPosition(double position) {
        return new RunCommand(() -> {
            targetPosition = Math.max(minPosition, Math.min(maxPosition, position));
            double output = pidController.calculate(
                hoodMotor.getPosition().getValueAsDouble(),
                targetPosition
            );
            hoodMotor.set(output);
        }, this);
    }

    // ==================== MANUAL CONTROL ====================

    /**
     * Manually runs the hood motor at a specified speed.
     * Use for testing, tuning, or finding position limits.
     *
     * @param speed Motor output from -1.0 to 1.0
     * @return Command that runs the hood at the specified speed
     */
    public Command runHood(double speed) {
        return new RunCommand(() -> {
            hoodMotor.set(speed);
        }, this);
    }

    /**
     * Runs the hood motor using a speed supplier.
     *
     * @param speedSupplier Supplier for the motor speed
     * @return Command that runs the hood at the supplied speed
     */
    public Command runHood(DoubleSupplier speedSupplier) {
        return new RunCommand(() -> {
            hoodMotor.set(speedSupplier.getAsDouble());
        }, this);
    }

    /**
     * Runs hood with auto-aim by default, switches to manual when override is enabled.
     *
     * @param overrideEnabled Supplier that returns true when manual control is desired
     * @param manualSpeed Supplier for manual control speed (right stick Y)
     * @return Command that switches between auto-aim and manual control
     */
    public Command runHoodWithOverride(BooleanSupplier overrideEnabled, DoubleSupplier manualSpeed) {
        return new RunCommand(() -> {
            if (overrideEnabled.getAsBoolean()) {
                // Manual control when override (Y button) is held
                hoodMotor.set(manualSpeed.getAsDouble());
            } else {
                // Auto-aim using odometry-based distance to goal
                double distance = vision.getDistanceToGoal();
                targetPosition = getHoodPositionForDistance(distance);

                double output = pidController.calculate(
                    hoodMotor.getPosition().getValueAsDouble(),
                    targetPosition
                );
                hoodMotor.set(output);
            }
        }, this);
    }

    /**
     * Continuously commands the hood motor to stop.
     * Use as a default command to hold position when not adjusting.
     *
     * @return RunCommand that continuously sets the motor to zero
     */
    public Command stopAll() {
        return new RunCommand(() -> {
            hoodMotor.set(0.0);
        }, this);
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Gets the current hood position.
     *
     * @return Current position in motor rotations
     */
    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    /**
     * Checks if the hood has reached the target position.
     *
     * @return True if within tolerance of target position
     */
    public boolean atTarget() {
        return pidController.atSetpoint();
    }

    // ==================== HOMING ====================

    /**
     * Resets the hood position by moving down until the limit switch triggers.
     * Zeros the encoder at the known home position.
     *
     * @return Command that homes the hood and resets the encoder
     */
    public Command calibrateHood() {
        return new RunCommand(() -> {
            hoodMotor.set(0.15);
        }, this)
            .until(() -> (hoodMotor.getStatorCurrent().getValueAsDouble() > HoodConstants.HOMING_STALL_AMPS))
            .andThen(new InstantCommand(() -> {
                hoodMotor.set(0.0);
                hoodMotor.setPosition(0.0);
                isCalibrated = true;
            }));
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }   

    @Override
    public void periodic() {
        // Publish telemetry to SmartDashboard
        SmartDashboard.putNumber("Hood/Position", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Target", targetPosition);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
        SmartDashboard.putNumber("Hood/StallCurrent", hoodMotor.getStatorCurrent().getValueAsDouble());
        // Display current distance to goal for tuning the lookup table
        SmartDashboard.putNumber("Hood/DistanceToGoal", vision.getDistanceToGoal());
        SmartDashboard.putBoolean("Hood/IsCalibrated", isCalibrated);
    }
}
