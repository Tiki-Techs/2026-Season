package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Index subsystem that controls the indexer/feeder mechanism.
 * Transfers game pieces from the intake to the shooter using belt-driven rollers.
 * Uses a TalonFX motor (CAN ID 25) for belt control.
 */
public class Index extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Index motor that drives the belt system */
    private final TalonFX intake25 = new TalonFX(IndexConstants.INDEX_MOTOR);

    // ==================== CONTROL COMMANDS ====================

    /**
     * Runs the indexer using controller trigger input for variable speed.
     * Speed is proportional to right trigger position.
     *
     * @param controllerValue Xbox controller to read trigger axis from
     * @return Command that continuously runs the indexer based on trigger input
     */
    public Command runIndex(CommandXboxController controllerValue) {
        return new RunCommand(() ->
            intake25.set(controllerValue.getRightTriggerAxis()),
            this
        );
    }

    /**
     * Runs the indexer in reverse using controller trigger input.
     * Useful for unjamming or repositioning game pieces.
     *
     * @param controllerValue Xbox controller to read trigger axis from
     * @return Command that runs the indexer in reverse based on trigger input
     */
    public Command runReverseIndex(CommandXboxController controllerValue) {
        return new RunCommand(() ->
            intake25.set(-controllerValue.getRightTriggerAxis()),
            this
        );
    }

    /**
     * Runs the indexer at a fixed speed.
     *
     * @param speed Motor output from -1.0 (reverse) to 1.0 (forward)
     * @return Command that continuously runs the indexer at the specified speed
     */
    public Command runIndex(double speed) {
        return new RunCommand(() ->
            intake25.set(speed),
            this
        );
    }

    // ==================== STOP COMMANDS ====================

    /**
     * Immediately stops the indexer motor (instant command).
     * Executes once and completes.
     *
     * @return InstantCommand that stops the indexer motor
     */
    public Command stopIndex() {
        return new InstantCommand(() ->
            intake25.set(0),
            this
        );
    }

    /**
     * Continuously commands the indexer motor to stop.
     * Use as a default command to ensure the motor stays stopped when not in use.
     * The "this" parameter ensures only one command can control this subsystem at a time.
     *
     * @return RunCommand that continuously sets the motor to zero
     */
    public Command stopAll() {
        return new RunCommand(() -> {
            intake25.set(0);
        }, this);
    }

    @Override
    public void periodic() {
        // Periodic updates (telemetry can be added here)
    }
}
