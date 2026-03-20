package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR, "CANivore");
    private final Vision vision;
    private final PIDController pidController = new PIDController(0.1, 0, 0);
    private final InterpolatingDoubleTreeMap distanceToHoodPosition = new InterpolatingDoubleTreeMap();

    private double minPosition = 0.0;
    private double maxPosition = 5.0;
    private double targetPosition = 0.0;
    private boolean isCalibrated = false;

    public Hood(Vision vision) {
        this.vision = vision;

        var currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(15)
            .withSupplyCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(currentLimits));
        // Distance (meters) to hood position lookup table
        distanceToHoodPosition.put(2.88, 0.0);
        distanceToHoodPosition.put(4.1, 0.0);
        distanceToHoodPosition.put(1.9685, 0.0);
        distanceToHoodPosition.put(3.1877, 0.0);
        
        pidController.setTolerance(0.25);
    }
    
        /** Calibrates the hood by moving down until stall detected, then zeros encoder. */
        public Command calibrateHood() {
            final Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);
            final int[] loopCount = {0};
    
            return new SequentialCommandGroup(
                new InstantCommand(() -> loopCount[0] = 0),
    
                new RunCommand(() -> {
                    loopCount[0]++;
                    hoodMotor.set(-0.15);
                }, this).until(() -> {
                    if (loopCount[0] < 25) return false;
                    return stallDebouncer.calculate(hoodMotor.getStatorCurrent().getValueAsDouble() > HoodConstants.HOMING_STALL_AMPS);
                }),
    
                new InstantCommand(() -> {
                    hoodMotor.set(0.0);
                    hoodMotor.setPosition(0.0);
                    isCalibrated = true;
                }, this)
            );
        }
    
        public boolean isCalibrated() {
            return isCalibrated;
        }
    
    /** Gets the ideal hood position for a given distance in meters. */
    public double getHoodPositionForDistance(double distanceMeters) {
        double position = distanceToHoodPosition.get(distanceMeters);
        return Math.max(maxPosition, Math.min(minPosition, position));
    }

    private void setHoodMotorSafe(double speed) {
        if (hoodMotor.getPosition().getValueAsDouble() >= maxPosition-.2 && speed > 0) {
            hoodMotor.set(0);
            return;
        }
        hoodMotor.set(speed);
    }

    /** Automatically adjusts hood angle based on distance to goal using odometry. */
    public Command autoAimHood() {
        return new RunCommand(() -> {
            double distance = vision.getDistanceToGoal();
            targetPosition = getHoodPositionForDistance(distance);
            double output = pidController.calculate(hoodMotor.getPosition().getValueAsDouble(), targetPosition);
            setHoodMotorSafe(output);
        }, this);
    }

    /** Moves the hood to a specific encoder position using PID control. */
    public Command setPosition(double position) {
        return new RunCommand(() -> {
            targetPosition = Math.max(minPosition, Math.min(maxPosition, position));
            double output = pidController.calculate(hoodMotor.getPosition().getValueAsDouble(), targetPosition);
            setHoodMotorSafe(output);
        }, this);
    }

    /** Manually runs the hood motor at a specified speed. Stops if stall detected. */
    public Command runHood(double speed) {
        return new RunCommand(() -> setHoodMotorSafe(speed), this);
    }

    /** Continuously stops the hood motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> hoodMotor.set(0.0), this);
    }

    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hood/IsCalibrated", isCalibrated);
        SmartDashboard.putNumber("Hood/Position", hoodMotor.getPosition().getValueAsDouble());
    }
}
