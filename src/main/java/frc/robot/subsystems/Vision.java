package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;



public class Vision extends SubsystemBase {

  private final SwerveSubsystem drivetrain;
  private final Field2d m_field = new Field2d();
  private final StructPublisher<Pose2d> posePublisher;

    public Vision(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        SmartDashboard.putData("Field", m_field);

        // Publish pose for AdvantageScope 3D field
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("RobotPose", Pose2d.struct).publish();
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
    double error = ty + 6.0;  // stops at TY = -8.0
    double targetingForwardSpeed = error * kP;

    return targetingForwardSpeed;
  }
    

    

    @Override
    public void periodic() {
        // Update field pose for Shuffleboard and AdvantageScope
        Pose2d currentPose = drivetrain.getState().Pose;
        m_field.setRobotPose(currentPose);
        posePublisher.set(currentPose);

        boolean tv = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");

        System.out.println("TV: " + tv + " TX: " + tx + " TY: " + ty);

        if(tv) {
            LimelightHelpers.setLEDMode_ForceBlink("limelight");
        } else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }


        
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2 == null || mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  

    }
}
