package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/** Lowers the pivot arm to the deployed position and waits until it arrives (encoder-based). */
public class PivotCommandAuto extends SequentialCommandGroup {

    public PivotCommandAuto(Pivot pivot) {
        addCommands(
            pivot.lowerArmManual(PivotConstants.LOWER_SPEED)
                .until(pivot::isAtLowerLimit)
                .withTimeout(3.0) // Safety timeout if encoder is uncalibrated
        );
    }
}
