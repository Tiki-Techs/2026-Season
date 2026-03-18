package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

/** Controls the flywheel shooter with three TalonFX motors. Supports open-loop and PID velocity control. */
public class Shooter extends SubsystemBase {

    private final TalonFX shooterOne = new TalonFX(ShooterConstants.FLOOR_ONE, "CANivore");
    private final TalonFX shooterTwo = new TalonFX(ShooterConstants.FLOOR_TWO, "CANivore");
    private final TalonFX shooterThree = new TalonFX(ShooterConstants.FLOOR_THREE, "CANivore");

    private final StatusSignal<AngularVelocity> velocityOne;
    private final StatusSignal<AngularVelocity> velocityTwo;
    private final StatusSignal<AngularVelocity> velocityThree;

    private final InterpolatingDoubleTreeMap distanceToShooterSpeed = new InterpolatingDoubleTreeMap();
    private final double minSpeed = 40.0;
    private final double maxSpeed = 100.0;

    private double shooterTargetVelocity = 0;
    private final VelocityVoltage shooterVoltageRequest = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);

    public Shooter() {
        velocityOne = shooterOne.getVelocity();
        velocityTwo = shooterTwo.getVelocity();
        velocityThree = shooterThree.getVelocity();

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.KS;
        slot0Configs.kV = ShooterConstants.KV;
        slot0Configs.kP = ShooterConstants.KP;
        slot0Configs.kI = ShooterConstants.KI;
        slot0Configs.kD = ShooterConstants.KD;

        shooterOne.getConfigurator().apply(slot0Configs);
        shooterTwo.getConfigurator().apply(slot0Configs);
        shooterThree.getConfigurator().apply(slot0Configs);

        // Distance (meters) to shooter speed (RPS) lookup table
        // TODO: Calibrate these values by testing at known distances
        distanceToShooterSpeed.put(2.88, 67.5);
        distanceToShooterSpeed.put(4.1, 82.5);
        distanceToShooterSpeed.put(1.9685, 55.5);
        distanceToShooterSpeed.put(3.1877, 72.5);
    }

    /** Gets the ideal shooter speed for a given distance in meters. */
    public double getShooterSpeedForDistance(double distanceMeters) {
        double speed = distanceToShooterSpeed.get(distanceMeters);
        return Math.max(minSpeed, Math.min(maxSpeed, speed));
    }

    /** Automatically adjusts shooter speed based on distance to goal. */
    public Command autoAimShooter(DoubleSupplier distanceSupplier) {
        return new RunCommand(() -> {
            double distance = distanceSupplier.getAsDouble();
            double targetRPS = getShooterSpeedForDistance(distance);
            shooterOne.setControl(shooterVoltageRequest.withVelocity(targetRPS).withFeedForward(0.5));
            shooterTwo.setControl(shooterVoltageRequest.withVelocity(targetRPS).withFeedForward(0.5));
            shooterThree.setControl(shooterVoltageRequest.withVelocity(targetRPS).withFeedForward(0.5));
        }, this);
    }

    /** Runs the shooter at a target velocity using PID control. */
    public Command runPIDShooter(double targetRPS) {
        return new RunCommand(() -> {
            shooterOne.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
            shooterTwo.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
            shooterThree.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
        }, this);
    }

    /** Runs the shooter using trigger input for variable speed (open-loop). */
    public Command runShooter(CommandXboxController controllerValue) {
        return new RunCommand(() -> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooterOne.set(speed);
            shooterTwo.set(speed);
            shooterThree.set(speed);
        }, this);
    }

    /** Runs the shooter at a fixed speed (open-loop). */
    public Command runShooter(double speed) {
        return new RunCommand(() -> {
            shooterOne.set(-speed);
            shooterTwo.set(-speed);
            shooterThree.set(-speed);
        }, this);
    }

    /** Runs the shooter in reverse using trigger input. */
    public Command runReverseShooter(CommandXboxController controllerValue) {
        return new RunCommand(() -> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooterOne.set(-speed);
            shooterTwo.set(-speed);
            shooterThree.set(-speed);
        }, this);
    }

    /** Immediately stops all shooter motors. */
    public Command stopShooter() {
        return new InstantCommand(() -> {
            shooterOne.set(0);
            shooterTwo.set(0);
            shooterThree.set(0);
        }, this);
    }

    /** Continuously stops all shooter motors. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> {
            shooterOne.set(0);
            shooterTwo.set(0);
            shooterThree.set(0);
        }, this);
    }

    public void setShooterTargetVelocity(double velocityRPS) {
        this.shooterTargetVelocity = velocityRPS;
    }

    /** Checks if all shooter motors are within tolerance of target speed. */
    public boolean isAtTargetSpeed(double targetRPS, double tolerance) {
        BaseStatusSignal.refreshAll(velocityOne, velocityTwo, velocityThree);
        double currentOneVelocity = Math.abs(velocityOne.getValueAsDouble());
        double currentTwoVelocity = Math.abs(velocityTwo.getValueAsDouble());
        double currentThreeVelocity = Math.abs(velocityThree.getValueAsDouble());
        return Math.abs(currentOneVelocity - Math.abs(targetRPS)) <= tolerance &&
               Math.abs(currentTwoVelocity - Math.abs(targetRPS)) <= tolerance &&
               Math.abs(currentThreeVelocity - Math.abs(targetRPS)) <= tolerance;
    }

    /** Checks if shooter is at auto-aim target speed for given distance. */
    public boolean isAtAutoAimTargetSpeed(double distanceMeters, double tolerance) {
        double targetRPS = getShooterSpeedForDistance(distanceMeters);
        return isAtTargetSpeed(targetRPS, tolerance);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(velocityOne, velocityTwo, velocityThree);
        double vel1 = Math.abs(velocityOne.getValueAsDouble());
        double vel2 = Math.abs(velocityTwo.getValueAsDouble());
        double vel3 = Math.abs(velocityThree.getValueAsDouble());
        double avgVelocity = (vel1 + vel2 + vel3) / 3.0;

        SmartDashboard.putNumber("Shooter/TargetVelocity", shooterTargetVelocity);
        SmartDashboard.putNumber("Shooter/Motor1/Velocity", vel1);
        SmartDashboard.putNumber("Shooter/Motor2/Velocity", vel2);
        SmartDashboard.putNumber("Shooter/Motor3/Velocity", vel3);
        SmartDashboard.putNumber("Shooter/AvgVelocity", avgVelocity);
        SmartDashboard.putBoolean("Shooter/AtTargetSpeed", isAtTargetSpeed(shooterTargetVelocity, 2.0));
    }
}
