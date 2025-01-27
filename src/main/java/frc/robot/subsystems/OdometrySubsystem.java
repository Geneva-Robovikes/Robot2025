// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OdometrySubsystem extends SubsystemBase {
  /** Creates a new OdometrySubsystem. */
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final AprilTagFieldLayout aprilTagLayout;

  public OdometrySubsystem() {
    camera = new PhotonCamera(Constants.VisionConstants.kCameraOne);
    aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    photonEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateVisionOdometry() {
    /* TODO: Investigate "Camera Matrix" */
    photonEstimator.update(camera.getLatestResult());
  }
}