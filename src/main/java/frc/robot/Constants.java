// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static boolean overrideEnabled = false; 

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // public static final int kMechControllerPort = 1;

    
  }
  
  public static class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public static final double kMaxAngularSpeedRadiansPerSecond = 
      RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

  // Swerve system IDs, (DO NOT USE THESE AGAIN)
  // Drive motors 1, 4, 7, 10
  // Angle motors 2, 5, 8, 11
  // Encoders     3, 6, 9, 12

  public static final class ClimbConstants{
    private ClimbConstants(){};

    public static final int CLIMB_MOTOR = 30;  // NEVER TESTED YET<<<<
    public static final int LOWER_LIMIT_SWITCH = 4; // NEVER TESTED YET<<<<
    public static final int UPPER_LIMIT_SWITCH = 5; // NEVER TESTED YET<<<<
  }

  public static final class HoodConstants{
    private HoodConstants(){};

    public static final int HOOD_MOTOR = 12; 
    public static final int LOWER_LIMIT_SWITCH = 5; 
  }

// Motor constants for intake
  public static final class IndexConstants{
    private IndexConstants(){};

    public static final int INDEX_MOTOR = 26; // FIX THIS <<<< CONFLICTING DOUBLE CHECK EVERYTHING

    public static final double INDEX_SPEED = 1.00;  // Change index speed here. 
  }

  // Motor constants for intake
  public static final class IntakeConstants{
    private IntakeConstants(){};

    public static final int LEADER_INTAKE = 25; 

    public static final double INTAKE_SPEED = 0.60;  // Change intake speed here. 
  }

  // Motor and limit switch constants for intake pivot (arm is way better name)
  public static final class IntakePivotConstants{
    private IntakePivotConstants(){};

    public static final int PIVOT_ARM = 24;
    public static final int LOWER_LIMIT_SWITCH = 3; 
    public static final int UPPER_LIMIT_SWITCH = 2; 

    public static final double INTAKE_PIVOT_SPEED = 0.25; // ACCORDING TO SOMEONE, this is normal speed (.25), it was set at .50 at my current time
  } 

  // Constants for shooter, 
  public static final class ShooterConstants{
    private ShooterConstants(){};

    public static final int CENTER_SHOOTER_ID = 21;  
    public static final int LEFT_SHOOTER_ID = 22; // NOT CONFIRMED YET, PLEASE DOUBLE CHECK.  <<<<
    public static final int RIGHT_SHOOTER_ID = 23;  // NOT CONFIRMED YET, PLEASE DOUBLE CHECK.  <<<<

    public static final double BANG_BANG_ON_POWER = 1.0;
    public static final double BANG_BANG_OFF_POWER = 0.0;

    public static final double PID_TARGET_RPS = 100.0;     // Change PID speed here. 
    public static final double SHOOTER_DEFAULT_SPEED = 1.0;     // Change shooter default speed here. 
  }

  public static final class ShooterIntakeConstants{
    private ShooterIntakeConstants(){};

    public static final double SHOOTER_INTAKE_SPEED = 1.0;
    public static final int SHOOTER_INTAKE_ID = 22; // CURRENTLY CONFLICTING ID, CHANGE IMMEDIATELY <<<<
  } 


}
