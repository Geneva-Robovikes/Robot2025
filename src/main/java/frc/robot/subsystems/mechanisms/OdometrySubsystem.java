// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OdometrySubsystem extends SubsystemBase {
  private final PhotonCamera frontLeft;
  private final PhotonCamera frontRight;
  private final PhotonCamera backLeft;
  private final PhotonCamera backRight;

  private final AprilTagFieldLayout fieldLayout;

  private final Transform3d frontLeftPosition;
  private final Transform3d frontRightPosition;
  private final Transform3d backLeftPosition;
  private final Transform3d backRightPosition;

  private final PhotonPoseEstimator photonPoseEstimatorFrontLeft;
  private final PhotonPoseEstimator photonPoseEstimatorFrontRight;
  private final PhotonPoseEstimator photonPoseEstimatorBackLeft;
  private final PhotonPoseEstimator photonPoseEstimatorBackRight;

  private List<EstimatedRobotPose> estimatedPoses = new ArrayList<EstimatedRobotPose>();

  
  public OdometrySubsystem() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    frontLeft = new PhotonCamera(Constants.VisionConstants.kFrontLeft);
    frontRight = new PhotonCamera(Constants.VisionConstants.kFrontRight);
    backLeft = new PhotonCamera(Constants.VisionConstants.kBackLeft);
    backRight = new PhotonCamera(Constants.VisionConstants.kBackRight);

    frontLeftPosition = Constants.VisionConstants.kFrontLeftPosition;
    frontRightPosition = Constants.VisionConstants.kFrontRightPosition;
    backLeftPosition = Constants.VisionConstants.kBackLeftPosition;
    backRightPosition = Constants.VisionConstants.kBackRightPosition;

    photonPoseEstimatorFrontLeft = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftPosition);
    photonPoseEstimatorFrontRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightPosition);
    photonPoseEstimatorBackLeft = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftPosition);
    photonPoseEstimatorBackRight = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightPosition);
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> photonEstimatedPoseFrontLeft = photonPoseEstimatorFrontLeft.update(frontLeft.getLatestResult());
    Optional<EstimatedRobotPose> photonEstimatedPoseFrontRight = photonPoseEstimatorFrontRight.update(frontRight.getLatestResult());
    Optional<EstimatedRobotPose> photonEstimatedPoseBackLeft = photonPoseEstimatorBackLeft.update(backLeft.getLatestResult());
    Optional<EstimatedRobotPose> photonEstimatedPoseBackRight = photonPoseEstimatorBackRight.update(backRight.getLatestResult());

    if (photonEstimatedPoseFrontLeft.isPresent()) {
      estimatedPoses.add(photonEstimatedPoseFrontLeft.get());
    }
    if (photonEstimatedPoseFrontRight.isPresent()) {
      estimatedPoses.add(photonEstimatedPoseFrontRight.get());
    }
    if (photonEstimatedPoseBackLeft.isPresent()) {
      estimatedPoses.add(photonEstimatedPoseBackLeft.get());
    }
    if (photonEstimatedPoseBackRight.isPresent()) {
      estimatedPoses.add(photonEstimatedPoseBackRight.get());
    }
  }

  public List<EstimatedRobotPose> getEstimatedRobotPoses() {
    return estimatedPoses;
  }
}
