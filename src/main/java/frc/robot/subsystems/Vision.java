// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  // private PhotonCamera camera = new PhotonCamera(VisionConstants.photonCameraName);
  //   private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
  //           FieldConstants.APRIL_TAG_FIELD_LAYOUT,
  //           PoseStrategy.AVERAGE_BEST_TARGETS,
  //           camera,
  //           VisionConstants.photonRobotToCamera);
  // /** Creates a new Vision. */
  // public Vision(BiConsumer<Pose2d, Double> addVisionMeasurement, Supplier<Pose2d> robotPoseSupplier) {
  //   this.addVisionMeasurement = addVisionMeasurement;
  //   this.robotPoseSupplier = robotPoseSupplier;
  //   setLimelightMode(limelightMode);
}
