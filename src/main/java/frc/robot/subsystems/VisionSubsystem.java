// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

  private final PhotonPoseEstimator photonPoseEstimatorCameraOne;
  private final PhotonPoseEstimator photonPoseEstimatorCameraTwo;
  private final PhotonPoseEstimator photonPoseEstimatorCameraThree;

  public VisionSubsystem() {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    cameraOne = new PhotonCamera(Constants.VisionConstants.kCameraOne);
    cameraTwo = new PhotonCamera(Constants.VisionConstants.kCameraTwo);
    cameraThree = new PhotonCamera(Constants.VisionConstants.kCameraThree);

    cameraOnePosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));
    cameraTwoPosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));
    cameraThreePosition = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));

    photonPoseEstimatorCameraOne = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraOnePosition);
    photonPoseEstimatorCameraTwo = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraTwoPosition);
    photonPoseEstimatorCameraThree = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cameraThreePosition);
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

  /* Returns an optional estimated pose; used for telling where the robot is on the field based on what the cameras see. 
   * 
   * I do admit that this is a bit ridiculous.
  */
  public List<Optional<EstimatedRobotPose>> getEstimatedPose() {
    List<Optional<EstimatedRobotPose>> estimatedPoses = new ArrayList<Optional<EstimatedRobotPose>>();

    Optional<EstimatedRobotPose> photonEstimatedPoseCamOne = photonPoseEstimatorCameraOne.update(cameraOne.getLatestResult());
    Optional<EstimatedRobotPose> photonEstimatedPoseCamTwo = photonPoseEstimatorCameraTwo.update(cameraTwo.getLatestResult());
    Optional<EstimatedRobotPose> photonEstimatedPoseCamThree = photonPoseEstimatorCameraThree.update(cameraThree.getLatestResult());

    var camOneResult = cameraOne.getLatestResult();
    var camTwoResult = cameraOne.getLatestResult();
    var camThreeResult = cameraOne.getLatestResult();

    boolean camOneHasTargets = camOneResult.hasTargets();
    boolean camTwoHasTargets = camTwoResult.hasTargets();
    boolean camThreeHasTargets = camThreeResult.hasTargets();

    if (camOneHasTargets) {
      if (photonEstimatedPoseCamOne.isPresent()){
        EstimatedRobotPose estimatedPose = photonEstimatedPoseCamOne.get();

        estimatedPoses.add(Optional.of(estimatedPose));
      } else {
        estimatedPoses.add(Optional.empty());
      }
    } else {
      estimatedPoses.add(Optional.empty());
    }

    if (camTwoHasTargets) {
      if (photonEstimatedPoseCamTwo.isPresent()){
        EstimatedRobotPose estimatedPose = photonEstimatedPoseCamTwo.get();

        estimatedPoses.add(Optional.of(estimatedPose));
      } else {
        estimatedPoses.add(Optional.empty());
      }
    } else {
      estimatedPoses.add(Optional.empty());
    }

    if (camThreeHasTargets) {
      if (photonEstimatedPoseCamThree.isPresent()){
        EstimatedRobotPose estimatedPose = photonEstimatedPoseCamThree.get();

        estimatedPoses.add(Optional.of(estimatedPose));
      } else {
        estimatedPoses.add(Optional.empty());
      }
    } else {
      estimatedPoses.add(Optional.empty());
    }

    return estimatedPoses;
  }
}
