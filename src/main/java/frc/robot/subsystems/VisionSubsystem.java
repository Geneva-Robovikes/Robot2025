// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.kCameraName);

  public VisionSubsystem() {}

  @Override
  public void periodic() {}

  /* Returns an "optional" PhotonTrackedTarget. Basically, if there is no target, it returns null. */
  public Optional<PhotonTrackedTarget> getTarget() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      return Optional.of(target);
    }

    return Optional.empty();
  }
}
