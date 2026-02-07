// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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


    
  }
  
  public static class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(17.1);
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /0.449;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularAccelerationRadiansPerSecond = 3.5;

    // PID Gains for X and Y position control (TUNING REQUIRED)
    public static final double kPXController = 0.5; 
    public static final double kPYController = 0.5; 
    
    // Profiled PID Gains for Theta (Rotation) control (TUNING REQUIRED)
    public static final double kPThetaController = 1.0;
    
  }
}
