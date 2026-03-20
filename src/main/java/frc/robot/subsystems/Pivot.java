package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    private final double stallCurrentThreshold = PivotConstants.HOMING_STALL_LOWER_AMPS;
    private final double stopSpeed = 0.0;
    private final double totalTravelDistance = 7.5;

    private boolean intakeDeployed = true;
    private boolean isCalibrated = false;
    private double lowerEncoderPos = 0.0;
    private double upperEncoderPos = 0.0;
    private int pivotLoopCounter = 0;

    /** Reference to climb for safety interlock - pivot cannot go up while climb is down. */
    private Climb climb = null;

    public Pivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);
        pivotArm.configureAsync(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
    }



    /** Calibrates the pivot by finding the lower hard stop via stall detection, then raising. */
    public Command calibratePivot() {
        final Debouncer phase1Stall = new Debouncer(0.15, Debouncer.DebounceType.kRising);
        final int[] loopCount = {0};

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                isCalibrated = false;
                loopCount[0] = 0;
            }, this),

            // Find lower hard stop
            new RunCommand(() -> {
                loopCount[0]++;
                pivotArm.set(-PivotConstants.HOMING_SPEED);
            }, this).until(() -> {
                if (loopCount[0] < 25) return false;
                return phase1Stall.calculate(pivotArm.getOutputCurrent() > PivotConstants.HOMING_STALL_LOWER_AMPS);
            }),

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

    public boolean isCalibrated() {
        return isCalibrated;
    }

    /** Returns true if the pivot is in the down/deployed position. */
    public boolean isPivotDown() {
        return intakeDeployed;
    }

    /** Sets the climb reference for safety interlock. Must be called after construction. */
    public void setClimb(Climb climb) {
        this.climb = climb;
    }

    private boolean canRaise() {
        return climb == null || !climb.isClimbDown();
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
            if (!canRaise()) {
                pivotArm.set(stopSpeed);
                return;
            }

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


    @Override
    public void periodic() {
        pivotLoopCounter++;
        SmartDashboard.putBoolean("Pivot/IsCalibrated", isCalibrated);
        SmartDashboard.putBoolean("Pivot/IntakeDeployed", intakeDeployed);
        SmartDashboard.putNumber("Pivot/EncoderPosition", encoder.getPosition());
        SmartDashboard.putBoolean("Pivot/CanRaise", canRaise());
       }
}
