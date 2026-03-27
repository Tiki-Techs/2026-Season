package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

/** Controls the intake arm pivot mechanism. Raises and lowers the intake between deployed and stowed positions. */
public class Pivot extends SubsystemBase {

    private final SparkMax pivotArm = new SparkMax(PivotConstants.PIVOT_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder encoder = pivotArm.getEncoder();
    private final DigitalInput lowerLimitSwitch = new DigitalInput(PivotConstants.LOWER_LIMIT_SWITCH_DIO);
    private final double stopSpeed = 0.0;
    private final double totalTravelDistance = 7.5;

    private boolean intakeDeployed = true;
    private boolean isCalibrated = false;
    private double lowerEncoderPos = 0.0;
    private double upperEncoderPos = 0.0;

    public Pivot() {}



    /** Calibrates the pivot by finding the lower limit switch, then zeroing the encoder. */
    public Command calibratePivot() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                isCalibrated = false;
            }, this),

            // Move down until limit switch is triggered
            new RunCommand(() -> {
                pivotArm.set(PivotConstants.HOMING_SPEED);
            }, this).until(this::isLowerLimitPressed),

            // Zero encoder and set calibration state
            new InstantCommand(() -> {
                pivotArm.set(stopSpeed);
                encoder.setPosition(0.0);
                lowerEncoderPos = 0.0;
                upperEncoderPos = -totalTravelDistance;
                intakeDeployed = true;
                isCalibrated = true;
            }, this)
        );
    }

    /** Returns true if the lower limit switch is pressed (switch is normally open, so false = pressed). */
    public boolean isLowerLimitPressed() {
        return !lowerLimitSwitch.get();
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    /** Returns true if the pivot is in the down/deployed position. */
    public boolean isPivotDown() {
        return intakeDeployed;
    }

    /** Continuously stops the pivot motor. */
    public Command stopAll() {
        return new RunCommand(() -> pivotArm.set(stopSpeed), this);
    }

    /** Manually lowers the intake arm. Slows near bottom and stops at limit. */
    public Command lowerArmManual(double pivotSpeed) {
        final double slowSpeed = 0.05;

        return new RunCommand(() -> {
            double distanceToLower = Math.abs(encoder.getPosition() - lowerEncoderPos);
            boolean atLimit = isCalibrated && distanceToLower < 0.05;

            if (atLimit) {
                pivotArm.set(stopSpeed);
            } else {
                boolean inSlowZone = isCalibrated && distanceToLower < (totalTravelDistance * 0.4);
                pivotArm.set(inSlowZone ? slowSpeed : pivotSpeed);
            }
        }, this);
    }

    /** Manually raises the intake arm. Slows near top and stops at limit. */
    public Command raiseArmManual(double pivotSpeed) {
        final double slowSpeed = 0.2;

        return new RunCommand(() -> {
            double distanceToUpper = Math.abs(encoder.getPosition() - upperEncoderPos);
            boolean atLimit = isCalibrated && distanceToUpper < 0.05;

            if (atLimit) {
                pivotArm.set(stopSpeed);
            } else {
                boolean inSlowZone = isCalibrated && distanceToUpper < (totalTravelDistance * 0.4);
                pivotArm.set(inSlowZone ? -slowSpeed : -pivotSpeed);
            }
        }, this);
    }


    public Command runPivot(double pivotSpeed) {
        return new RunCommand(() -> {
            pivotArm.set(pivotSpeed);
        }, this);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pivot/IsCalibrated", isCalibrated);
        SmartDashboard.putBoolean("Pivot/IntakeDeployed", intakeDeployed);
        SmartDashboard.putNumber("Pivot/EncoderPosition", encoder.getPosition());
        SmartDashboard.putBoolean("Pivot/LowerLimitSwitch", isLowerLimitPressed());
    }
}
