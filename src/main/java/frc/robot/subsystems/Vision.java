// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.Collections;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.BiConsumer;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.numbers.N5;
// import edu.wpi.first.math.numbers.N7;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.VisionConstants;

 public class Vision extends SubsystemBase {
//   // private PhotonCamera camera = new PhotonCamera(VisionConstants.photonCameraName);
//   //   private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
//   //           FieldConstants.APRIL_TAG_FIELD_LAYOUT,
//   //           PoseStrategy.AVERAGE_BEST_TARGETS,
//   //           camera,
//   //           VisionConstants.photonRobotToCamera);
//   // /** Creates a new Vision. */
//   // public Vision(BiConsumer<Pose2d, Double> addVisionMeasurement, Supplier<Pose2d> robotPoseSupplier) {
//   //   this.addVisionMeasurement = addVisionMeasurement;
//   //   this.robotPoseSupplier = robotPoseSupplier;
//   //   setLimelightMode(limelightMode);
  
//     private PhotonCamera photonCamera;
//     private  Swerve swerve;
    
//     private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
//         new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, Math.toRadians(180.0))),
//         new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, Math.toRadians(180.0)))));;
    
//     private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5),0.05,0.05,0.05,0.05);

//     private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01),0.01,Math.toRadians(5),0.01,0.01);

//     private static final Vector<N3>  visionMeasurementDevs = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(10));

//     private SwerveDrivePoseEstimator poseEstimator;

//     private final Field2d field2d = new Field2d();
    
//     private double previousPipelineTimestamp = 0;
    

//     public void PoseEstimatorSubsystem(PhotonCamera photonCamera, Swerve swerve){
//         this.photonCamera = photonCamera;
//         this.swerve = swerve;

//         ShuffleboardTab tab = Shuffleboard.getTab("Vision ");

//         poseEstimator = new SwerveDrivePoseEstimator(
//           Nat.N7(),
//           Nat.N7(),
//           Nat.N5(),
//           swerve.getGyroscopicRotation(),
//           swerve.getSwerveState().getSwerveModulePositions(),
//           new Pose2d(),
//           Constants.Swerve.swerveKinematics,
//           stateStdDevs,
//           localMeasurementStdDevs,
//           visionMeasurementDevs
//         );

//         tab.addString("Pose",  this::getFormattedPose).withPosition(0,0);
//         tab.add("Field",field2d).withPosition(2, 0).withSize(10, 10);
//     }

//     @Override
//     public void periodic(){
//       var pipelineResult = photonCamera.getLatestResult();
//       var resultTimestamp = pipelineResult.getTimestampSeconds();
//       if(resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()){
//         previousPipelineTimestamp = resultTimestamp;
//         var target = pipelineResult.getBestTarget();
//         var fiducialID = target.getFiducialId();
//         if (target.getPoseAmbiguity() <= .2 && fiducialID >= 0 && targetPoses.size() > fiducialID ){
//             var targetPose = targetPoses.get(fiducialID);
//             Transform3d camToTarget = target.getBestCameraToTarget();
//             Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

//             var visionMeasurement = camPose.transformBy(camToTarget);
//             poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
//         }

//       }

//       var drivetrainState = swerve.getDrivetrainState();
//       poseEstimator.update(swerve.getGyroscopicRotation(), swerve.getModuleStates(), swerve.getModulePositions());
//       field2d.setRobotPose(getCurrentPose());
//     }


//     private String getFormattedPose(){
//       var pose = getCurrentPose();
//       return String.format("(%.2f, %.2f) %.2f degrees");
//     }

//     public Pose2d getCurrentPose(){
//       return poseEstimator.getEstimatedPosition();
//     }


 }
  

