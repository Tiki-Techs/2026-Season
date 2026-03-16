package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

/**
 * Pivot subsystem that controls the intake arm pivot mechanism.
 * Raises and lowers the intake between deployed (ground pickup) and stowed positions.
 * Uses a SparkMax controller with NEO brushless motor and limit switches for position limits.
 */
public class Pivot extends SubsystemBase {

    // ==================== HARDWARE ====================

    /** Pivot arm motor - SparkMax with NEO brushless motor */
    private final SparkMax pivotArm = new SparkMax(PivotConstants.PIVOT_MOTOR, MotorType.kBrushless);

    /** Encoder for position tracking */
    private final RelativeEncoder encoder = pivotArm.getEncoder();

    /** Stall current threshold for detecting hard stops (lowering) */
    private final double stallCurrentThreshold = PivotConstants.HOMING_STALL_LOWER_AMPS;

    // ==================== CONSTANTS ====================

    /** Speed value used to stop the pivot motor */
    private final double stopSpeed = 0.0;

    // ==================== STATE ====================

    /** Tracks whether the intake is currently deployed (lowered) or stowed (raised) */
    private boolean intakeDeployed = true;

    /**
     * Total travel distance for the pivot in motor rotations.
     * Note: Motor has 25:1 gear ratio, so this is motor rotations not output shaft rotations.
     * Calibrated by zeroing encoder at bottom limit and measuring encoder at top limit.
     */
    private double totalTravelDistance = 7.5;


    private boolean isCalibrated = false;

    private double lowerEncoderPos = 0.0;
    private double upperEncoderPos = 0.0;



    /**
     * Constructs the Pivot subsystem and configures the motor.
     * Sets the motor to brake mode so the arm holds position when stopped.
     */
    public Pivot() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
    }

    // ==================== HOLD/STOP COMMANDS ====================

    /** Power to apply to hold the arm up against gravity (tune this value) */
    private final double holdUpPower = -0.05;

    /**
     * Default command that holds the arm in position.
     * When stowed (up), applies power to counteract gravity.
     * When deployed (down), no power needed as gravity keeps it down.
     *
     * @return RunCommand that actively holds position
     */
    public Command holdPosition() {
        return new RunCommand(() -> {
            if (!intakeDeployed) {
                // Arm is up - apply holding power against gravity
                // Use encoder to check if near upper limit
                boolean atUpperLimit = isCalibrated && Math.abs(encoder.getPosition() - upperEncoderPos) < 0.1;
                if (!atUpperLimit) {
                    pivotArm.set(holdUpPower);
                } else {
                    pivotArm.set(stopSpeed);
                }
            } else {
                // Arm is down - no power needed
                pivotArm.set(stopSpeed);
            }
        }, this);
    }


public Command calibratePivot() {
        // Debouncer for stall detection - 0.15s sustained current
        final Debouncer phase1Stall = new Debouncer(0.15, Debouncer.DebounceType.kRising);
        // Counter to skip stall detection for first 0.5s (25 loops at 50Hz)
        final int[] loopCount = {0};

        return new SequentialCommandGroup(

            // Reset calibration state
            new InstantCommand(() -> {
                isCalibrated = false;
                loopCount[0] = 0;
                SmartDashboard.putString("Pivot/CalibrationPhase", "Starting calibration...");
            }),

            // ---- Find the lower hard stop ----
            new RunCommand(() -> {
                loopCount[0]++;
                pivotArm.set(PivotConstants.HOMING_SPEED);
            }, this)
            .until(() -> {
                // Skip first 25 loops (0.5s) to let motor get moving
                if (loopCount[0] < 25) return false;
                return phase1Stall.calculate(
                    pivotArm.getOutputCurrent() > PivotConstants.HOMING_STALL_LOWER_AMPS
                );
            }),

            // ---- Zero encoder and set calibration state ----
            new InstantCommand(() -> {
                pivotArm.set(stopSpeed);
                // Zero encoder at bottom
                encoder.setPosition(0.0);
                lowerEncoderPos = 0.0;
                // Use fixed travel distance - upper limit is negative (going up)
                upperEncoderPos = -totalTravelDistance;
                intakeDeployed = true;
                isCalibrated = true;

                SmartDashboard.putNumber("Pivot/EncoderValue", encoder.getPosition());
                SmartDashboard.putNumber("Pivot/LowerLimit", lowerEncoderPos);
                SmartDashboard.putNumber("Pivot/UpperLimit", upperEncoderPos);
                SmartDashboard.putNumber("Pivot/TotalTravel", totalTravelDistance);
                SmartDashboard.putBoolean("Pivot/IsCalibrated", true);
                SmartDashboard.putString("Pivot/CalibrationPhase", "Calibration complete!");
            }, this)
        );
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    /**
     * Returns true if the pivot is in the down/deployed position.
     * Used for climb safety interlock.
     */
    public boolean isPivotDown() {
        return intakeDeployed;
    }


    /**
     * Continuously commands the pivot motor to stop.
     * Use when you explicitly want no motor output.
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
     * Slows down near the bottom to prevent slamming into the limit.
     * Stops automatically when stall is detected or encoder limit reached.
     *
     * @param pivotSpeed Base speed for lowering (will be reduced near limit)
     * @return Command that lowers the arm until released or limit reached
     */
    public Command lowerArmManual(double pivotSpeed) {
        final double slowSpeed = 0.05;
        final Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        return new RunCommand(() -> {
            boolean stalled = stallDebouncer.calculate(pivotArm.getOutputCurrent() > stallCurrentThreshold);
            double distanceToLower = Math.abs(encoder.getPosition() - lowerEncoderPos);
            boolean atLimit = isCalibrated && distanceToLower < 0.05;

            if (stalled || atLimit) {
                pivotArm.set(stopSpeed);
                if (stalled) {
                    intakeDeployed = true;
                }
            } else {
                // Slow zone: within 40% of total travel from bottom
                boolean inSlowZone = isCalibrated && distanceToLower < (totalTravelDistance * 0.4);

                double targetSpeed = inSlowZone ? slowSpeed : .075;
                pivotArm.set(targetSpeed);
            }
        }, this);
    }

    /**
     * Manually raises the intake arm while button is held.
     * Slows down near the top to prevent slamming into the limit.
     * Stops automatically when stall is detected or encoder limit reached.
     *
     * @param pivotSpeed Base speed for raising (will be reduced near limit)
     * @return Command that raises the arm until released or limit reached
     */
    public Command raiseArmManual(double pivotSpeed) {
        final double slowSpeed = 0.2;
        final Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

        return new RunCommand(() -> {
            // Use higher threshold for raising (fighting gravity draws more current)
            boolean stalled = stallDebouncer.calculate(pivotArm.getOutputCurrent() > PivotConstants.HOMING_STALL_RAISE_AMPS);
            double distanceToUpper = Math.abs(encoder.getPosition() - upperEncoderPos);
            boolean atLimit = isCalibrated && distanceToUpper < 0.05;

            if (atLimit) {
                pivotArm.set(stopSpeed);
                if (stalled) {
                    intakeDeployed = false;
                }
            } else {
                // Slow zone: within 40% of total travel from top
                boolean inSlowZone = isCalibrated && distanceToUpper < (totalTravelDistance * 0.4);

                double targetSpeed = inSlowZone ? -slowSpeed : -1.0;
                pivotArm.set(targetSpeed);
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
     * Automatically lowers the intake arm until stall detected or encoder limit reached.
     * Updates the deployed state when complete.
     *
     * @return Command that lowers to deployed position and updates state
     */
    public Command lowerArmAuto() {
        final double fastSpeed = 0.6;
        final double slowSpeed = 0.15;
        final Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);

        return new RunCommand(() -> {
            // Slow zone: within 40% of total travel from bottom
            double distanceToLower = Math.abs(encoder.getPosition() - lowerEncoderPos);
            boolean inSlowZone = isCalibrated && distanceToLower < (totalTravelDistance * 0.4);

            double targetSpeed = inSlowZone ? slowSpeed : fastSpeed;
            pivotArm.set(targetSpeed);

        }, this)
            .until(() -> {
                boolean stalled = stallDebouncer.calculate(pivotArm.getOutputCurrent() > stallCurrentThreshold);
                double distanceToLower = Math.abs(encoder.getPosition() - lowerEncoderPos);
                boolean atLimit = isCalibrated && distanceToLower < 0.05;
                return stalled || atLimit;
            })
            .unless(() -> isCalibrated && Math.abs(encoder.getPosition() - lowerEncoderPos) < 0.05)
            .finallyDo(interrupted -> {
                pivotArm.set(stopSpeed);
                if (!interrupted) {
                    intakeDeployed = true;
                }
            });
    }

    /**
     * Automatically raises the intake arm until stall detected or encoder limit reached.
     * Slows down near the top for a gentle approach.
     * Updates the deployed state when complete.
     *
     * @return Command that raises to stowed position and updates state
     */
    public Command raiseArmAuto() {
        final double fastSpeed = 0.6;
        final double slowSpeed = 0.15;
        final Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);

        return new RunCommand(() -> {
            // Slow zone: within 40% of total travel from top
            double distanceToUpper = Math.abs(encoder.getPosition() - upperEncoderPos);
            boolean inSlowZone = isCalibrated && distanceToUpper < (totalTravelDistance * 0.4);

            double targetSpeed = inSlowZone ? -slowSpeed : -fastSpeed;
            pivotArm.set(targetSpeed);

        }, this)
            .until(() -> {
                // Use higher threshold for raising (fighting gravity draws more current)
                boolean stalled = stallDebouncer.calculate(pivotArm.getOutputCurrent() > PivotConstants.HOMING_STALL_RAISE_AMPS);
                double distanceToUpper = Math.abs(encoder.getPosition() - upperEncoderPos);
                boolean atLimit = isCalibrated && distanceToUpper < 0.05;
                return stalled || atLimit;
            })
            .unless(() -> isCalibrated && Math.abs(encoder.getPosition() - upperEncoderPos) < 0.05)
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
        double traveled = Math.abs(encoder.getPosition());
        SmartDashboard.putNumber("Pivot/EncoderPosition", encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Halfway", totalTravelDistance / 2.0);
        SmartDashboard.putBoolean("Pivot/PastHalfway", traveled > totalTravelDistance / 2.0);
        SmartDashboard.putBoolean("Pivot/Deployed", intakeDeployed);
    
        SmartDashboard.putBoolean("Pivot/IsCalibrated", isCalibrated);
        SmartDashboard.putNumber("Pivot/HomingCurrent", pivotArm.getOutputCurrent());
        SmartDashboard.putNumber("Pivot/LowerLimit", lowerEncoderPos);
        SmartDashboard.putNumber("Pivot/UpperLimit", upperEncoderPos);
    }
}
