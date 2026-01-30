package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;



public class Vision extends SubsystemBase {

    double ty = LimelightHelpers.getTY("limelight");
    
    public Vision() {

    }


    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
  public double limelight_aim_proportional() {
    // kP (constant of proportionality) in (rad/s per radian of error)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    // Now that we're using proper radian conversion, kP should be higher (typically 2-5)
    double kP = 1.5;

    // Only aim if we have a valid target
    if (!LimelightHelpers.getTV("limelight")) {
      return 0.0;
    }

    double tx = LimelightHelpers.getTX("limelight");

    // Add a deadband to ignore small errors
    if (Math.abs(tx) < 5.0) {
      return 0.0;
    }

    // Convert TX from degrees to radians
    double txRadians = Math.toRadians(tx);

    // Apply proportional control to get target angular velocity in rad/s
    double targetingAngularVelocity = txRadians * kP;


    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional() {
    // kP in (m/s per degree of TY) - tune this value
    // Higher = more aggressive forward driving
    double kP = 0.2;

    // Only drive if we have a valid target
    if (!LimelightHelpers.getTV("limelight")) {
      return 0.0;
    }

    // TY is in degrees - we'll use it directly as an error signal
    // Negative TY = target below center = drive forward
    double ty = LimelightHelpers.getTY("limelight");
    double targetingForwardSpeed = ty * kP;

    return targetingForwardSpeed;
  }
    

    

    @Override
    public void periodic() {
        boolean tv = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        ty = LimelightHelpers.getTY("limelight");

        System.out.println("TV: " + tv + " TX: " + tx + " TY: " + ty);

        if(tv) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }

    }
}
