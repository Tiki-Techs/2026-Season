package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.Set;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem that controls the roller mechanism for collecting game pieces.
 * Uses a SparkFlex controller with a NEO Vortex brushless motor (CAN ID 25).
 * The intake rollers pull game pieces into the robot for transfer to the indexer.
 */
public class Intake extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Main intake roller motor - SparkFlex with NEO Vortex brushless motor */
    private final SparkFlex leaderIntake = new SparkFlex(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

    // ==================== CONSTANTS ====================

    /** Speed value used to stop the intake motor */
    private final double stopSpeed = 0.0;

    // ==================== CONTROL COMMANDS ====================

    /**
     * Runs the intake rollers at a specified speed.
     * Positive values intake game pieces, negative values eject.
     *
     * @param speed Motor output from -1.0 (eject) to 1.0 (intake)
     * @return Command that continuously runs the intake at the specified speed
     */
    public Command runIntake(double speed) {
        return new RunCommand(() -> {
            leaderIntake.set(speed);
        }, this);
    }

    // ==================== STOP COMMANDS ====================

    /**
     * Continuously commands the intake motor to stop.
     * Use as a default command to ensure the motor stays stopped when not in use.
     *
     * @return RunCommand that continuously sets the motor to zero
     */
    public Command stopAll() {
        return new RunCommand(() -> {
            leaderIntake.set(stopSpeed);
        }, this);
    }

    /**
     * Stops the intake motor.
     * Identical to stopAll() - provided for API consistency.
     *
     * @return RunCommand that continuously sets the motor to zero
     */
    public Command stopIntake() {
        return new RunCommand(() -> {
            leaderIntake.set(0);
        }, this);
    }

    @Override
    public void periodic() {
        // Periodic updates (telemetry can be added here)
    }
}
