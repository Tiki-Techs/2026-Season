package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Utility class for creating slow drive commands.
 * Reduces robot speed for testing, tuning, or precise maneuvering.
 *
 * Usage: Bind slowDown() to a button or use as an alternative default command
 * when reduced speed is needed (e.g., during demos or tight spaces).
 */
public class SlowDriveTrain {

    /**
     * Creates a command that drives the robot at reduced speed.
     * Applies a 10% scaling to translational velocity and 2.5% to rotational velocity.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param currentMaxSpeed The normal maximum translational speed (m/s)
     * @param currentMaxAngularRate The normal maximum rotational speed (rad/s)
     * @param joystick The Xbox controller for input
     * @return A command that drives at reduced speed while active
     */
    public Command slowDown(
            SwerveSubsystem drivetrain,
            double currentMaxSpeed,
            double currentMaxAngularRate,
            CommandXboxController joystick) {

        // Create a new field-centric drive request
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

        return drivetrain.applyRequest(() -> {
            // Speed scaling factors
            double vscale = 0.1;       // 10% of normal translational speed
            double vrotation = 0.025;  // 2.5% of normal rotational speed

            return drive
                .withVelocityX(-joystick.getLeftY() * currentMaxSpeed * vscale)
                .withVelocityY(-joystick.getLeftX() * currentMaxSpeed * vscale)
                .withRotationalRate(-joystick.getRightX() * currentMaxAngularRate * vrotation);
        });
    }
}
