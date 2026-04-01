package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/** Spins up shooter then feeds game piece. Runs shooter at full target RPS (long range). Times out after 5 seconds. */
public class ShootCommandAutoLong extends SequentialCommandGroup {

    public ShootCommandAutoLong(Shooter shooter, Index index, Feeder feeder, Vision vision) {
                addCommands(
            shooter.autoAimShooter(() -> vision.getDistanceToGoal())
                    .until(() -> shooter.isAtAutoAimTargetSpeed(vision.getDistanceToGoal(), 8.0)),
            new ParallelCommandGroup(
                shooter.autoAimShooter(() -> vision.getDistanceToGoal()),
                index.runIndex(IndexConstants.INDEX_SPEED),
                feeder.runFeeder(-FeederConstants.FEEDER_SPEED)
            )
                .withTimeout(5.0)
                .andThen(
                    shooter.stopShooter()
                        .alongWith(index.stopIndex())
                )
        );
    }
}
