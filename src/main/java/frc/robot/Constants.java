// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final int kAuxiliaryControllerPort = 1;
    public static final double controllerDeadzone = 0.06;
  }
  
  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 150.0/7.0;
    public static final double kFalconEncoderResolution = 2048;

    /* These are the only variables that change the max speed and acceleration! */
    public static final double kMaxSpeedMetersPerSecond = 7.0;
    public static final double kMaxAccelMetersPerSecond = 6.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;

    public static final double kDistanceBetweenWheels = Units.inchesToMeters(18.5);


    // Deadzone constant for controller
    public static final double kDeadzoneMinimum = .1;
    public static final double kDeadzoneMaximum = .9;

    /* Locations of swerve modules in relation to the robots center. */
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kDistanceBetweenWheels/2, kDistanceBetweenWheels/2), //front left
      new Translation2d(kDistanceBetweenWheels/2, -kDistanceBetweenWheels/2), //front right
      new Translation2d(-kDistanceBetweenWheels/2, kDistanceBetweenWheels/2), //back left
      new Translation2d(-kDistanceBetweenWheels/2, -kDistanceBetweenWheels/2)); //back right


    /* Standard deviations used in the SwerveDrivePoseEstimator, the higher n1 and n2, the more the pose estimator
     * trusts its output. The visions standard deviation should be fairly high until we do some calibration to get 
     * the exact error.
     */
    public static final Vector<N3> kStateStdDev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Vector<N3> kVisionStdDev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));
  }

  public static class VisionConstants {
    public static final String kCameraOne = "cam1";
    public static final String kCameraTwo = "cam2";
    public static final String kCameraThree = "cam3";

    public static final Transform3d kCameraOnePosition = new Transform3d(new Translation3d(Units.inchesToMeters(-12.5), 0.0, 0), new Rotation3d(0,0,0));
    public static final Transform3d kCameraTwoPosition = new Transform3d(new Translation3d(Units.inchesToMeters(12.5), 0.0, 0), new Rotation3d(0,0,0));
    public static final Transform3d kCameraThreePosition = new Transform3d(new Translation3d(0, Units.inchesToMeters(-12.5), 0), new Rotation3d(0,0,0));

    public static final int[] kReefAprilTags = {11, 17, 18, 19, 20, 21, 22};

    /* TODO: Measure specific values */
    public static final double kLoadingYawOffset = 0.1;
    public static final double kReefYawOffset = 0-12;

    public static final double kReefDistanceOffset = 1.9;
    public static final double kLoadingDistanceOffset = 1;

    public static final double kMaxVisionAlignmentSpeed = 6;
    public static final double kMaxVisionDistAlignmentSpeed = 6;
    public static final double kMaxVisionRotationalSpeed = 3.14;
    public static final double kMaxVisionRotationSpeed = Math.PI/2;

    public static final double kCameraHeight = Units.inchesToMeters(4.5);
    public static final double kReefAprilTagHeight = Units.inchesToMeters(19.4);
  }

  public static class MechanismConstants {
    /* TODO: real voltage value*/
    public static final double kClawMotorSpeed = .3;
    public static final double kMaxClawMotorCurrent = 0;
  }
}
