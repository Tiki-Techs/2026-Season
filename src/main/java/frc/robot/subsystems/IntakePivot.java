package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

/**
 * IntakePivot subsystem that controls the intake arm pivot mechanism.
 * Raises and lowers the intake between deployed (ground pickup) and stowed positions.
 * Uses a SparkMax controller with NEO brushless motor and limit switches for position limits.
 */
public class IntakePivot extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Pivot arm motor - SparkMax with NEO brushless motor */
    private final SparkMax pivotArm = new SparkMax(IntakePivotConstants.PIVOT_MOTOR, MotorType.kBrushless);

    /** Encoder for position tracking */
    private final RelativeEncoder encoder = pivotArm.getEncoder();

    /**
     * Lower limit switch.
     * Returns TRUE when unplugged or not triggered, FALSE when triggered.
     */
    private final DigitalInput lowerLimitSwitch = new DigitalInput(IntakePivotConstants.LOWER_LIMIT_SWITCH);

    /**
     * Upper limit switch.
     * Returns TRUE when unplugged or not triggered, FALSE when triggered.
     */
    private final DigitalInput upperLimitSwitch = new DigitalInput(IntakePivotConstants.UPPER_LIMIT_SWITCH);

    // ==================== CONSTANTS ====================

    /** Speed for raising/lowering the pivot arm (0.0 to 1.0) */
    private final double pivotSpeed = IntakePivotConstants.PIVOT_SPEED;

    /** Speed value used to stop the pivot motor */
    private final double stopSpeed = 0.0;

    // ==================== STATE ====================

    /** Tracks whether the intake is currently deployed (lowered) or stowed (raised) */
    private boolean intakeDeployed = true;

    /**
     * Total travel distance for the pivot in motor rotations.
     * Note: Motor has 25:1 gear ratio, so this is motor rotations not output shaft rotations.
     * Calibrated by zeroing encoder at bottom limit and measuring encoder at top limit.
     */
    private final double totalTravelDistance = 2.5;

    /**
     * Constructs the IntakePivot subsystem and configures the motor.
     * Sets the motor to brake mode so the arm holds position when stopped.
     */
    public IntakePivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        pivotArm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== HOLD/STOP COMMANDS ====================

    /** Power to apply to hold the arm up against gravity (tune this value) */
    private final double holdUpPower = -0.05;

    /**
     * Default command that holds the arm in position.
     * When stowed (up), applies power to counteract gravity.
     * When deployed (down), no power needed as gravity keeps it down.
     *
     * @return RunCommand that actively holds position
     */
    public Command holdPosition() {
        return new RunCommand(() -> {
            if (!intakeDeployed) {
                // Arm is up - apply holding power against gravity
                // Only apply if not already at upper limit
                if (upperLimitSwitch.get()) {
                    pivotArm.set(holdUpPower);
                } else {
                    pivotArm.set(stopSpeed);
                }
            } else {
                // Arm is down - no power needed
                pivotArm.set(stopSpeed);
            }
        }, this);
    }

    /**
     * Continuously commands the pivot motor to stop.
     * Use when you explicitly want no motor output.
     *
     * @return RunCommand that continuously sets the motor to zero
     */
    public Command stopAll() {
        return new RunCommand(() -> {
            pivotArm.set(stopSpeed);
        }, this);
    }

    // ==================== MANUAL CONTROL ====================

    /**
     * Manually lowers the intake arm while button is held.
     * Slows down near the bottom to prevent slamming into the limit.
     * Stops automatically when the lower limit switch is triggered.
     *
     * @param pivotSpeed Base speed for lowering (will be reduced near limit)
     * @return Command that lowers the arm until released or limit reached
     */
    public Command lowerArmManual(double pivotSpeed) {
        final double slowSpeed = 0.15;  // Speed near the limit (soft landing)
        final double slowZoneThreshold = 0.25; // Slow down when below 25% (near bottom)

        return new RunCommand(() -> {
            if (!lowerLimitSwitch.get()) {
                // Limit switch triggered - stop the motor
                pivotArm.set(stopSpeed);
            } else {
                // Calculate speed based on position
                // progress = 1.0 at top, 0.0 at bottom
                double traveled = Math.abs(encoder.getPosition());
                double progress = Math.min(traveled / totalTravelDistance, 1.0);

                double targetSpeed;
                if (progress > slowZoneThreshold) {
                    // Above 25% - fast zone
                    targetSpeed = pivotSpeed;
                } else {
                    // Below 25% - slow zone near bottom limit
                    targetSpeed = slowSpeed;
                }

                pivotArm.set(targetSpeed);
            }
        }, this);
    }

    /**
     * Manually raises the intake arm while button is held.
     * Slows down near the top to prevent slamming into the limit.
     * Stops automatically when the upper limit switch is triggered.
     *
     * @param pivotSpeed Base speed for raising (will be reduced near limit)
     * @return Command that raises the arm until released or limit reached
     */
    public Command raiseArmManual(double pivotSpeed) {
        final double slowSpeed = 0.15;  // Speed near the limit (soft landing)
        final double slowZoneStart = 0.75; // Start slowing at 75% of travel

        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()) {
                // Limit switch triggered - stop the motor
                pivotArm.set(stopSpeed);
            } else {
                // Calculate speed based on position
                double traveled = Math.abs(encoder.getPosition());
                double progress = Math.min(traveled / totalTravelDistance, 1.0);

                double targetSpeed;
                if (progress < slowZoneStart) {
                    // Fast zone - use requested speed
                    targetSpeed = -pivotSpeed;
                } else {
                    // Slow zone - gentle approach to limit switch
                    targetSpeed = -slowSpeed;
                }

                pivotArm.set(targetSpeed);
            }
        }, this);
    }

    // ==================== AUTOMATIC CONTROL ====================

    /**
     * Toggles the arm between deployed and stowed positions.
     * Checks current state and runs the appropriate auto command.
     *
     * @return ConditionalCommand that raises or lowers based on current state
     */
    public Command toggleArm() {
        return new ConditionalCommand(
            raiseArmAuto(),   // Run when intakeDeployed = true
            lowerArmAuto(),   // Run when intakeDeployed = false
            () -> intakeDeployed
        );
    }

    /**
     * Automatically lowers the intake arm until the lower limit switch triggers.
     * Updates the deployed state when complete.
     * Will not run if already at the lower limit.
     *
     * @return Command that lowers to deployed position and updates state
     */
    public Command lowerArmAuto() {
        return new RunCommand(() -> {
            pivotArm.set(pivotSpeed);
        }, this)
            .until(() -> !lowerLimitSwitch.get())    // Stop when limit triggered
            .unless(() -> !lowerLimitSwitch.get())   // Don't run if already at limit
            .finallyDo(interrupted -> {
                pivotArm.set(stopSpeed);
                if (!interrupted) {
                    intakeDeployed = true;
                    encoder.setPosition(0);  // Zero encoder at deployed position
                }
            });
    }

    /**
     * Automatically raises the intake arm until the upper limit switch triggers.
     * Starts at 0.25 speed, slows down after traveling halfway.
     * Updates the deployed state when complete.
     * Will not run if already at the upper limit.
     *
     * @return Command that raises to stowed position and updates state
     */
    public Command raiseArmAuto() {
        final double fastSpeed = 0.6;   // Speed for most of travel
        final double slowSpeed = 0.15;  // Speed near the limit (soft landing)
        final double slowZoneStart = 0.75; // Start slowing at 75% of travel

        return new RunCommand(() -> {
            // Use absolute encoder position since we zero at bottom
            double traveled = Math.abs(encoder.getPosition());
            double progress = Math.min(traveled / totalTravelDistance, 1.0);

            double targetSpeed;
            if (progress < slowZoneStart) {
                // Fast zone - full speed
                targetSpeed = -fastSpeed;
            } else {
                // Slow zone - gentle approach to limit switch
                targetSpeed = -slowSpeed;
            }

            pivotArm.set(targetSpeed);

            // Debug output
            SmartDashboard.putNumber("IntakePivot/RawEncoder", encoder.getPosition());
            SmartDashboard.putNumber("IntakePivot/Traveled", traveled);
            SmartDashboard.putNumber("IntakePivot/Progress", progress);
            SmartDashboard.putNumber("IntakePivot/TargetSpeed", targetSpeed);
            SmartDashboard.putBoolean("IntakePivot/InSlowZone", progress >= slowZoneStart);
        }, this)
            .until(() -> !upperLimitSwitch.get())    // Stop when limit triggered
            .unless(() -> !upperLimitSwitch.get())   // Don't run if already at limit
            .finallyDo(interrupted -> {
                pivotArm.set(stopSpeed);
                if (!interrupted) {
                    intakeDeployed = false;
                }
            });
    }

    // ==================== STATE MANAGEMENT ====================

    /**
     * Manually toggles the deployed state flag.
     * Use to resync state if the arm was moved manually or by external factors.
     *
     * @return InstantCommand that inverts the deployed state
     */
    public Command changeDeployState() {
        return new InstantCommand(() -> {
            intakeDeployed = !intakeDeployed;
        });
    }

    @Override
    public void periodic() {
        double traveled = Math.abs(encoder.getPosition());
        SmartDashboard.putNumber("IntakePivot/EncoderPosition", encoder.getPosition());
        SmartDashboard.putNumber("IntakePivot/TravelDistance", traveled);
        SmartDashboard.putNumber("IntakePivot/Halfway", totalTravelDistance / 2.0);
        SmartDashboard.putBoolean("IntakePivot/PastHalfway", traveled > totalTravelDistance / 2.0);
        SmartDashboard.putBoolean("IntakePivot/Deployed", intakeDeployed);
    }
}
