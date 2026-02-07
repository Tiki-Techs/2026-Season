// Java: New Command File (e.g., frc.robot.commands.DriveToPointCommand.java)

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class DriveToPointCommand {

    /**
     * Creates a command to drive the robot from its current MegaTag2-corrected position to a target field coordinate.
     * @param targetPose The field coordinate (X, Y, and desired final Rotation) to drive to.
     * @param swerveSubsystem The SwerveSubsystem instance.
     * @return The SwerveControllerCommand ready to be scheduled.
     */
    public static SwerveControllerCommand getCommand(Pose2d targetPose, SwerveSubsystem swerveSubsystem) {
        
        // 1. Get the current, accurate starting position (corrected by MegaTag 2)
        Pose2d startPose = swerveSubsystem.getPose();

        // 2. Create the PID Controllers
        // These are the controllers that move the robot along the X and Y axes of the field.
        PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(DriveConstants.kPYController, 0, 0);
        
        // The profiled controller ensures the robot's rotation is smooth and respects kinematic limits.
        ProfiledPIDController thetaController = new ProfiledPIDController(
            DriveConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxAngularSpeedRadiansPerSecond, 
                DriveConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        );
        // Enable continuous input for the angle to handle 360-degree wrapping (e.g., going from 179 to -179 degrees)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        double maxSpeed = .1; //DriveConstants.kMaxSpeedMetersPerSecond * .25;
        double maxAccel = .05; //DriveConstants.kMaxAccelerationMetersPerSecondSquared * .25;

        // 3. Define the trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            maxSpeed, 
            maxAccel).setKinematics(swerveSubsystem.getKinematics());

        // Generate a straight line trajectory from startPose to targetPose
        // Since we only provide start and end, it's a simple path.
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startPose, 
            List.of(), // No interior waypoints
            targetPose, 
            config
        );
        
        // 4. Create the SwerveControllerCommand
        return new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose, // Supplier that gives the current pose (FUSION CORRECTED!)
            swerveSubsystem.getKinematics(),
            xController,
            yController,
            thetaController,
            () -> targetPose.getRotation(), // What angle the robot should face at the end
            swerveSubsystem::setModuleStates, // Consumer to execute the calculated speeds
            swerveSubsystem // Subsystems required by this command
        );
    }
}