package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IntakePivot subsystem that controls the intake arm pivot mechanism.
 * Raises and lowers the intake between deployed (ground pickup) and stowed positions.
 * Uses a SparkMax controller with NEO brushless motor and limit switches for position limits.
 */
public class IntakePivot extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Pivot arm motor - SparkMax with NEO brushless motor (CAN ID 24) */
    private final SparkMax pivotArm = new SparkMax(24, MotorType.kBrushless);

    /**
     * Lower limit switch (DIO port 3).
     * Returns TRUE when unplugged or not triggered, FALSE when triggered.
     */
    private final DigitalInput lowerLimitSwitch = new DigitalInput(3);

    /**
     * Upper limit switch (DIO port 2).
     * Returns TRUE when unplugged or not triggered, FALSE when triggered.
     */
    private final DigitalInput upperLimitSwitch = new DigitalInput(2);

    // ==================== CONSTANTS ====================

    /** Speed for raising/lowering the pivot arm (0.0 to 1.0) */
    private final double pivotSpeed = 0.25;

    /** Speed value used to stop the pivot motor */
    private final double stopSpeed = 0.0;

    // ==================== STATE ====================

    /** Tracks whether the intake is currently deployed (lowered) or stowed (raised) */
    private boolean intakeDeployed = true;

    /**
     * Constructs the IntakePivot subsystem and configures the motor.
     * Sets the motor to brake mode so the arm holds position when stopped.
     */
    public IntakePivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        pivotArm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== STOP COMMANDS ====================

    /**
     * Continuously commands the pivot motor to stop.
     * Use as a default command to hold the arm in place when not moving.
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
     * Stops automatically when the lower limit switch is triggered.
     * Use for testing or manual override situations.
     *
     * @return Command that lowers the arm until released or limit reached
     */
    public Command lowerArmManual() {
        return new RunCommand(() -> {
            if (!lowerLimitSwitch.get()) {
                // Limit switch triggered - stop the motor
                pivotArm.set(stopSpeed);
            } else {
                // Move down (positive direction)
                pivotArm.set(pivotSpeed);
            }
        }, this);
    }

    /**
     * Manually raises the intake arm while button is held.
     * Stops automatically when the upper limit switch is triggered.
     * Use for testing or manual override situations.
     *
     * @return Command that raises the arm until released or limit reached
     */
    public Command raiseArmManual() {
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()) {
                // Limit switch triggered - stop the motor
                pivotArm.set(stopSpeed);
            } else {
                // Move up (negative direction)
                pivotArm.set(-pivotSpeed);
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
                }
            });
    }

    /**
     * Automatically raises the intake arm until the upper limit switch triggers.
     * Updates the deployed state when complete.
     * Will not run if already at the upper limit.
     *
     * @return Command that raises to stowed position and updates state
     */
    public Command raiseArmAuto() {
        return new RunCommand(() -> {
            pivotArm.set(-pivotSpeed);
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
        // Periodic updates (telemetry can be added here)
    }
}
