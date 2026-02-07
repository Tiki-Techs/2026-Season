package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;



public class Shooter extends SubsystemBase{

    private final TalonFX shooter23 = new TalonFX(23);
    private final TalonFX shooter24 = new TalonFX(24);
    private final TalonFX shooter25 = new TalonFX(25);

    // Bang-bang control parameters
    private double shooterTargetVelocity = 0; // Target velocity in rotations per second
    private final double BANG_BANG_ON_POWER = 1.0; // Full power when below setpoint
    private final double BANG_BANG_OFF_POWER = 0.00; // Power when at/above setpoint

    final VelocityVoltage shooterVoltageRequest = new VelocityVoltage(0.0).withSlot(0);



    public Shooter() {
    //     // Configure shooter motors to coast mode for better bang-bang control
    //     shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    //     MotorFollower.setNeutralMode(NeutralModeValue.Coast);
    //     MotorFollower2.setNeutralMode(NeutralModeValue.Coast);

    //     // Intake can stay in brake mode for better control
    //     intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    //     // Configure current limiting to protect motors
    //     CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
    //         .withSupplyCurrentLimit(40)           // Supply current limit (amps)
    //         .withSupplyCurrentLimitEnable(true)   // Enable supply current limiting
    //         .withStatorCurrentLimit(60)           // Stator current limit (amps)
    //         .withStatorCurrentLimitEnable(true);  // Enable stator current limiting

    //     // Current limitors for bang bang testing
    //     shooterMotor.getConfigurator().apply(currentLimits);
    //     MotorFollower.getConfigurator().apply(currentLimits);
    //     MotorFollower2.getConfigurator().apply(currentLimits);
    //     intakeMotor.getConfigurator().apply(currentLimits);

        // in init function, set slot 0 gains
        // PID Controller for shooter
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        shooter23.getConfigurator().apply(slot0Configs);
        shooter24.getConfigurator().apply(slot0Configs);
        shooter25.getConfigurator().apply(slot0Configs);
        // Setting motor rotation orientation test this once we can get more time with the shooter. 
        // var motorConfig = new TalonFXConfiguration();
        // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // shooter23.getConfigurator().apply(motorConfig);
        // motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // shooter24.getConfigurator().apply(motorConfig);
        // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // shooter25.getConfigurator().apply(motorConfig);
    }

    public Command runPIDShooter(double targetRPS){
        return new RunCommand(()->{
            // motor id,  set controls, target RPS, with feedforward to overcome gravity and friction
            shooter23.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
            shooter24.setControl(shooterVoltageRequest.withVelocity(targetRPS).withFeedForward(0.5));
            shooter25.setControl(shooterVoltageRequest.withVelocity(-targetRPS).withFeedForward(0.5));
        }, this 
        );}

    public Command runShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooter23.set(-speed);
            shooter24.set(speed);
            shooter25.set(-speed);
        }
        ,this
        );
    };

    public Command runShooter(double speed){
        return new RunCommand(()-> {
            shooter23.set(-speed);
            shooter24.set(speed);
            shooter25.set(-speed);
        }
        ,this
        );
    };

    public Command runReverseShooter(CommandXboxController controllerValue){
        return new RunCommand(()-> {
            double speed = controllerValue.getLeftTriggerAxis();
            shooter23.set(speed);
            shooter24.set(-speed);
            shooter25.set(speed);
        }
        ,this
        );
    };

    // public Command runReverseShooter(double speed){
    //     return new RunCommand(()-> {
    //         shooter23.set(speed);
    //         shooter24.set(-speed);
    //         shooter25.set(speed);
    //     }
    //     ,this
    //     );
    // };
 
    public Command stopShooter(){
        return new InstantCommand(()->{
            shooter23.set(0);
            shooter24.set(0);
            shooter25.set(0);
        }
        ,this
        );
    }

    public Command stopAll(){
        return new RunCommand(()->{
                shooter23.set(0);
                shooter24.set(0);
                shooter25.set(0);

            },
            // ", this" makes sure that only the shooter subsystem object can only run command at a time
            this
        );
    }

    // Bang-bang control methods

    /**
     * Sets the target velocity for bang-bang control
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setShooterTargetVelocity(double velocityRPS) {
        this.shooterTargetVelocity = velocityRPS;
    }

    /**
     * Gets the current shooter velocity
     * @return Current velocity in rotations per second
     */
    public double getShooterVelocity() {
        return shooter23.getVelocity().getValueAsDouble();
    }

    /**
     * Applies bang-bang control to the shooter motor
     * Turns motor to full power if below setpoint, off if at/above setpoint
     */
    private void applyBangBangControl() {
        double currentVelocity = getShooterVelocity();

        if (currentVelocity < shooterTargetVelocity) {
            shooter23.set(-BANG_BANG_ON_POWER);
            shooter24.set(BANG_BANG_ON_POWER);
            shooter25.set(-BANG_BANG_ON_POWER);
        } else {
            shooter23.set(BANG_BANG_OFF_POWER);
            shooter24.set(BANG_BANG_OFF_POWER);
            shooter25.set(BANG_BANG_OFF_POWER);
        }
    }

    /**
     * Command to run shooter with bang-bang control at a target velocity
     * @param targetVelocityRPS Target velocity in rotations per second
     * @return Command that maintains the target velocity using bang-bang control
     */
    public Command runShooterBangBang(double targetVelocityRPS) {
        return new RunCommand(() -> {
            setShooterTargetVelocity(targetVelocityRPS);
            applyBangBangControl();
        }, this);
    }

    /**
     * Checks if the shooter is at the target velocity
     * @param tolerance Acceptable tolerance in rotations per second
     * @return True if within tolerance of target
     */
    public boolean isAtTargetVelocity(double tolerance) {
        return Math.abs(getShooterVelocity() - shooterTargetVelocity) <= tolerance;
    }

    @Override
    public void periodic(){
        

    }
}