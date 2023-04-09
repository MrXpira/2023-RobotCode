// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.swervedrive;

// import java.lang.reflect.Field;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.Robot;
// import frc.robot.Constants.Swerve;
// import frc.robot.LimelightHelpers.LimelightResults;

// public class PoseEstimator extends SubsystemBase {
//   private final SwerveDrivePoseEstimator poseEstimator;
//   private final SwerveSubsystem swerve;

//   /** Creates a new PoseEstimator. */
//   public PoseEstimator(SwerveSubsystem swerve) {
//     this.swerve = swerve;
//     poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, swerve.getYaw(), swerve.getModulePositions(), swerve.getPose());
//   }

//   public double[] visionPose() {
//     return DriverStation.getAlliance() == Alliance.Blue ? LimelightHelpers.getBotPose_wpiBlue("") : LimelightHelpers.getBotPose_wpiRed("");
//   }

  
//   private double getDistanceFromTarget() {
//     Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("");
//     return new Translation2d(pose.getX(), pose.getY()).getNorm();
//   }

//   private void updateVisionPose() {
//     if (Robot.isReal()) {
//       LimelightResults pipelineResults =  LimelightHelpers.getLatestResults("");
//       if (pipelineResults.targetingResults.valid) {
//         double[] botpose = visionPose();
//         double latency = (Timer.getFPGATimestamp() - (botpose[6]/1000.0));
        
//         Pose2d currentPose = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(Units.degreesToRadians(botpose[5])));
//         System.out.println(currentPose);
//         double trustWorthiness = 1;
//         // Apply vision measurements. For simulation purposes only, we don't input a latency delay -- on
//         // a real robot, this must be calculated based either on known latency or timestamps.
//         poseEstimator.addVisionMeasurement(currentPose, Timer.getFPGATimestamp(),
//         VecBuilder.fill(0.9, 0.9, 0.1).times(1.0 / trustWorthiness));
//       }
      
//    }

   
//    //poseEstimator.update(swerve.getYaw(), swerve.getModulePositions());
//   }
//   @Override
//   public void periodic(){
//     updateVisionPose();
//     System.out.println(getDistanceFromTarget() + " meters");    
//   }
// }
