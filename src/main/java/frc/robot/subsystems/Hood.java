// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;

// public class Hood extends SubsystemBase {

//     private final SparkFlex hoodMotor = new SparkFlex(30, MotorType.kBrushless);
//     private final RelativeEncoder encoder = hoodMotor.getEncoder();

//     // PID controller for position control - tune these values
//     private final PIDController pidController = new PIDController(0.1, 0, 0);

//     // Lookup table: TY (degrees) -> Hood position (encoder rotations)
//     // Add your tuned values here after testing
//     private final InterpolatingDoubleTreeMap tyToHoodPosition = new InterpolatingDoubleTreeMap();

//     // Hood limits (set these after testing)
//     private double minPosition = 0.0;  // TODO: Set after testing
//     private double maxPosition = 10.0; // TODO: Set after testing

//     private double targetPosition = 0.0;

//     public Hood() {
//         // Reset encoder on startup
//         encoder.setPosition(0);

//         // Placeholder lookup table values - TUNE THESE WITH REAL DATA
//         // Format: tyToHoodPosition.put(tyAngle, hoodEncoderPosition);
//         // Lower TY = farther away = higher hood angle
//         tyToHoodPosition.put(-20.0, 8.0);  // Far shot
//         tyToHoodPosition.put(-10.0, 5.0);  // Mid shot
//         tyToHoodPosition.put(0.0, 3.0);    // Close shot
//         tyToHoodPosition.put(10.0, 1.0);   // Very close

//         // PID tolerance
//         pidController.setTolerance(0.25);
//     }

//     /**
//      * Gets the hood position for a given TY angle from the lookup table
//      */
//     public double getHoodPositionForTY(double ty) {
//         double position = tyToHoodPosition.get(ty);
//         // Clamp to limits
//         return Math.max(minPosition, Math.min(maxPosition, position));
//     }

//     /**
//      * Command to automatically adjust hood based on limelight TY
//      */
//     public Command autoAimHood() {
//         return new RunCommand(() -> {
//             if (LimelightHelpers.getTV("limelight")) {
//                 double ty = LimelightHelpers.getTY("limelight");
//                 targetPosition = getHoodPositionForTY(ty);
//             }
//             double output = pidController.calculate(encoder.getPosition(), targetPosition);
//             hoodMotor.set(output);
//         }, this);
//     }

//     /**
//      * Command to move hood to a specific position
//      */
//     public Command setPosition(double position) {
//         return new RunCommand(() -> {
//             targetPosition = Math.max(minPosition, Math.min(maxPosition, position));
//             double output = pidController.calculate(encoder.getPosition(), targetPosition);
//             hoodMotor.set(output);
//         }, this);
//     }

//     /**
//      * Manual control for testing and finding limits
//      */
//     public Command runHood(double speed) {
//         return new RunCommand(() -> {
//             hoodMotor.set(speed);
//         }, this);
//     }

//     public Command stopAll() {
//         return new RunCommand(() -> {
//             hoodMotor.set(0);
//         }, this);
//     }

//     public double getPosition() {
//         return encoder.getPosition();
//     }

//     public boolean atTarget() {
//         return pidController.atSetpoint();
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Hood/Position", encoder.getPosition());
//         SmartDashboard.putNumber("Hood/Target", targetPosition);
//         SmartDashboard.putBoolean("Hood/AtTarget", atTarget());

//         // For tuning - read these and add to lookup table
//         if (LimelightHelpers.getTV("limelight")) {
//             SmartDashboard.putNumber("Hood/CurrentTY", LimelightHelpers.getTY("limelight"));
//         }
//     }
// }
