package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_MOTOR, "CANivore");
    private final DigitalInput upperLimitSwitch = new DigitalInput(ClimbConstants.UPPER_LIMIT_SWITCH);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(ClimbConstants.LOWER_LIMIT_SWITCH);
    private boolean isCalibrated = false;

    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true);
        climbMotor.getConfigurator().apply(config);
    }

    /** Continuously stops the climb motor. Use as default command. */
    public Command stopAll() {
        return new RunCommand(() -> climbMotor.set(0), this);
    }

    /** Calibrates the climb by finding the lower limit, then raising to the top. */
    public Command calibrateClimb() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {}, this),

            // Move up until upper limit switch is triggered (bypasses pivot interlock)
            new RunCommand(() -> {
                if (!upperLimitSwitch.get()) {
                    climbMotor.set(0);
                    return;
                }
                climbMotor.set(0.5);
            }, this).until(() -> !upperLimitSwitch.get()),

            // Zero encoder at bottom
            new InstantCommand(() -> {
                climbMotor.set(0.0);
                climbMotor.setPosition(0.0);
                isCalibrated = true;
            }, this)
            
            ,

            // Go back down
            new RunCommand(() -> {
                if (!lowerLimitSwitch.get()) {
                    climbMotor.set(0);
                } else {
                    climbMotor.set(-0.75);
                }
            }, this).until(() -> !lowerLimitSwitch.get()),

            new InstantCommand(() -> climbMotor.set(0.0), this)
        );
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    /** Gets the current climb position in motor rotations. */
    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }
    
    public Command getReady() {
        return new RunCommand(() -> {
            double currentPos = getPosition();
            if (currentPos < -9.0) {
                climbMotor.set(1.0);  // Move up if below target
            } else if (currentPos > -9.0) {
                climbMotor.set(-1.0); // Move down if above target
            } else {
                climbMotor.set(0);
            }
        }, this).until(() -> Math.abs(getPosition() - (-9.2)) < 0.5);
    }

    public Command climb() {
        return new RunCommand(() -> {
            climbMotor.set(1.0);
        }, this).until(() -> !lowerLimitSwitch.get());
    }


    public boolean isUpperLimitPressed() {
        return !upperLimitSwitch.get();
    }

    public boolean isLowerLimitPressed() {
        return !lowerLimitSwitch.get();
    }

    /** Returns true if the climb is at the lower limit (down position). */
    public boolean isClimbDown() {
        return !lowerLimitSwitch.get();
    }

    /** Runs the climb motor up. Stops at upper limit switch. */
    public Command runClimbUp() {
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1.0);
            }
        }, this);
    }

    public Command runHopperUp() {
        return new RunCommand(() -> {
            if (!upperLimitSwitch.get()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(1.0);
            }
        }, this);
    }

    /** Runs the climb motor down. */
    public Command runClimbDown() {
        return new RunCommand(() -> {
            if (!lowerLimitSwitch.get()) {
                climbMotor.set(0);
            } else {
                climbMotor.set(-1.0);
            }
        }, this);
    }

    public Command runHopperDown() {
        return new RunCommand(() -> {
            if(!lowerLimitSwitch.get()) {
                climbMotor.set(0);
                return;
            }
            climbMotor.set(-.1);
        }, this);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb/IsCalibrated", isCalibrated);
        SmartDashboard.putBoolean("Climb/UpperLimit", isUpperLimitPressed());
        SmartDashboard.putBoolean("Climb/LowerLimit", isLowerLimitPressed());
        SmartDashboard.putNumber("Climb/Position", climbMotor.getPosition().getValueAsDouble());
    }
}
