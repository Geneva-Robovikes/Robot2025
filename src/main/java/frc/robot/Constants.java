// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double controllerDeadzone = 0.1;
  }
  
  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.09677;
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 150.0/7.0;
    public static final double kFalconEncoderResolution = 2048;

    /* These are the only variables that change the max speed and acceleration! */
    public static final double kMaxSpeedMetersPerSecond = 6.0;
    public static final double kMaxAccelMetersPerSecond = 6.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;

    public static final double kDistanceBetweenWheels = Units.inchesToMeters(18.5);

    public static final double kTurnPIDkValue = 0.45;
    public static final double kDrivePIDkValue = 0.31;

    /* Locations of swerve modules in relation to the robots center. */
    /* TODO: Redo this so the front of the robot defaults to the roborio */
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kDistanceBetweenWheels/2, kDistanceBetweenWheels/2), //front left
      new Translation2d(-kDistanceBetweenWheels/2, -kDistanceBetweenWheels/2), //front right
      new Translation2d(kDistanceBetweenWheels/2, kDistanceBetweenWheels/2), //back left
      new Translation2d(kDistanceBetweenWheels/2, -kDistanceBetweenWheels/2)); //back right
    
  }

  public static class VisionConstants {
    public static final String kCameraName = "cam1";

    public static final int[] kReefAprilTags = {17, 18, 19, 20, 21, 22};

    /* TODO: Measure specific values */
    public static final double kLoadingYawOffset = 0.1;
    public static final double kReefYawOffset = 0.1;

    public static final double kReefDistanceOffset = 1;
    public static final double kLoadingDistanceOffset = 1;

    public static final double kMaxVisionAlignmentSpeed = 0.5;
  }
}
