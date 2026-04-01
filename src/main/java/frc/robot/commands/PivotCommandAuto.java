package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/** Lowers the pivot arm to the deployed position and waits until the lower limit is reached. */
public class PivotCommandAuto extends SequentialCommandGroup {

    public PivotCommandAuto(Pivot pivot) {
        addCommands(
            pivot.runPivot(PivotConstants.HOMING_SPEED)
            .until(pivot::isLowerLimitPressed)
        );
    }
}
