// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  public VisionSubsystem() {
    camera = new PhotonCamera(Constants.VisionConstants.kCameraName);
  }

  @Override
  public void periodic() {}

  public void getTargets() {
    var result = camera.getAllUnreadResults();

    PhotonPipelineResult target = result.get(0);
  }
}
