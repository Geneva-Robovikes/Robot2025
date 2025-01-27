// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera cameraOne;
  private final PhotonCamera cameraTwo;
  private final PhotonCamera cameraThree;

  private final AprilTagFieldLayout aprilTagFieldLayout;

  private final Transform3d cameraOnePosition;
  private final Transform3d cameraTwoPosition;
  private final Transform3d cameraThreePosition;

  private final PhotonPoseEstimator photonPoseEstimator;

  public VisionSubsystem() {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    cameraOne = new PhotonCamera(Constants.VisionConstants.kCameraOne);
    cameraTwo = new PhotonCamera(Constants.VisionConstants.kCameraTwo);
    cameraThree = new PhotonCamera(Constants.VisionConstants.kCameraThree);

    cameraOnePosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));
    cameraTwoPosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));
    cameraThreePosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraOnePosition);
  }

  @Override
  public void periodic() {}

  /* Returns an "optional" PhotonTrackedTarget. Basically, if there is no target, it returns null. */
  public Optional<PhotonTrackedTarget> getTarget() {
    var result = cameraOne.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      return Optional.of(target);
    }

    return Optional.empty();
  }
}
