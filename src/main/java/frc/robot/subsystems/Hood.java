package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.HoodConstants;

/**
 * Hood subsystem that controls the shooter hood angle for trajectory adjustment.
 * Uses a TalonFX motor to adjust the launch angle based on distance to target.
 * Supports automatic aiming using Limelight TY values with an interpolating lookup table.
 */
public class Hood extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Hood angle adjustment motor */
    private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR);

    /** Lower limit switch for hood zeroing */
    private final DigitalInput lowerLimitSwitch = new DigitalInput(HoodConstants.LOWER_LIMIT_SWITCH);

    // ==================== CONTROL ====================

    /** PID controller for closed-loop position control */
    private final PIDController pidController = new PIDController(0.1, 0, 0);

    /**
     * Lookup table mapping Limelight TY angles to hood motor positions.
     * Interpolates between data points for smooth hood adjustment.
     * Lower TY = farther target = higher hood angle needed.
     */
    private final InterpolatingDoubleTreeMap tyToHoodPosition = new InterpolatingDoubleTreeMap();

    // ==================== POSITION LIMITS ====================

    /** Minimum hood position (encoder rotations) - fully down */
    private double minPosition = 0.0;

    /** Maximum hood position (encoder rotations) - fully up */
    private double maxPosition = -1.1416015625;

    /** Current target position for PID control */
    private double targetPosition = 0.0;

    /**
     * Constructs the Hood subsystem and initializes configuration.
     * Zeros the motor encoder and populates the TY-to-position lookup table.
     */
    public Hood() {
        // Reset encoder position on startup
        hoodMotor.setPosition(0);

        // Populate lookup table with tuned values
        // Format: tyToHoodPosition.put(tyAngle, hoodEncoderPosition)
        // Lower TY = farther away = higher hood angle
        tyToHoodPosition.put(12.00, 8.0);  // Far shot
        tyToHoodPosition.put(-10.0, 5.0);  // Mid shot
        tyToHoodPosition.put(0.0, 3.0);    // Close shot
        tyToHoodPosition.put(10.0, 1.0);   // Very close

        // Set position tolerance for "at setpoint" checks
        pidController.setTolerance(0.25);
    }

    // ==================== LOOKUP TABLE ====================

    /**
     * Gets the ideal hood position for a given Limelight TY angle.
     * Uses interpolation between defined data points.
     *
     * @param ty Limelight TY value in degrees
     * @return Hood position in motor rotations, clamped to safe limits
     */
    public double getHoodPositionForTY(double ty) {
        double position = tyToHoodPosition.get(ty);
        return Math.max(minPosition, Math.min(maxPosition, position));
    }

    // ==================== AUTO-AIM ====================

    /**
     * Automatically adjusts hood angle based on Limelight target tracking.
     * Continuously reads TY, looks up the ideal position, and uses PID to reach it.
     *
     * @return Command that continuously auto-aims the hood while running
     */
    public Command autoAimHood() {
        return new RunCommand(() -> {
            // Only update target if we have a valid target
            if (LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME)) {
                double ty = LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
                targetPosition = getHoodPositionForTY(ty);
            }
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
    public Command resetPosition() {
        return new RunCommand(() -> {
            hoodMotor.set(-0.1);
        }, this)
            .until(() -> lowerLimitSwitch.get())
            .andThen(() -> {
                hoodMotor.set(0.0);
                hoodMotor.setPosition(0.0);
            });
    }

    @Override
    public void periodic() {
        // Publish telemetry to SmartDashboard
        SmartDashboard.putNumber("Hood/Position", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood/Target", targetPosition);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());

        // Display current TY for tuning the lookup table
        if (LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME)) {
            SmartDashboard.putNumber("Hood/CurrentTY", LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME));
        }
    }
}
