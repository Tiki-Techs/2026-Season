// package frc.robot.subsystems;
// import com.revrobotics.jni.CANSparkJNI;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.NativeResourceCleaner;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


 

// public class GroundIntake extends SubsystemBase{
//     private final SparkMax m_leader = new SparkMax(14, MotorType.kBrushless);
//     private final SparkMax m_follower = new SparkMax(15, MotorType.kBrushless);

//     private final double leaderSpeed = 0.5; 
//     private final double followerSpeed = -leaderSpeed;


//     public void runIntake(){
//         m_leader.set(leaderSpeed);
//         m_follower.set(followerSpeed);
//     }

//     public void stopIntake(){
//         m_leader.stopMotor();
//         m_follower.stopMotor();
//     }


//     public Command enableIntake(){
//         return runEnd(
//             this::runIntake,
//             this::stopIntake
//         );
//     }
// }
