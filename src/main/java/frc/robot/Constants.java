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

    public static final Vector<N3> kStateStdDev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  }

  public static class VisionConstants {
    public static final String kFrontLeft = "frontl";
    public static final String kFrontRight = "frontr";
    public static final String kBackLeft = "backl";
    public static final String kBackRight = "backr";

    public static final Transform3d kFrontLeftPosition = new Transform3d(new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(10.5), 0), new Rotation3d(0,0,Units.degreesToRadians(40)));
    public static final Transform3d kFrontRightPosition = new Transform3d(new Translation3d(Units.inchesToMeters(10.5), -Units.inchesToMeters(10.5), 0), new Rotation3d(0,0,Units.degreesToRadians(-40)));
    public static final Transform3d kBackLeftPosition = new Transform3d(new Translation3d(-Units.inchesToMeters(10.5), Units.inchesToMeters(10.5), 0), new Rotation3d(0,0,Units.degreesToRadians(130)));
    public static final Transform3d kBackRightPosition = new Transform3d(new Translation3d(-Units.inchesToMeters(10.5), -Units.inchesToMeters(10.5), 0), new Rotation3d(0,0,Units.degreesToRadians(-130)));

    public static final Vector<N3> kVisionStdDev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));
  }

  public static class MechanismConstants {
    /* TODO: real voltage value*/
    public static final double kIntakeMotorSpeed = .45;
    public static final double kIntakeTiltMotorSpeed = .1;
    public static final double kElevatorMotorDeadzone = .1;
    public static final double kElevatorMotorMaximumSpeed = .5;

    public static final double kIntakePivotMotorDownPosition = 0;
    public static final double kIntakePivotMotorUpPosition = -15;
    public static final double kIntakeL1MotorUpPosition = -11.52;
    public static final double kIntakeL1AutoMotorUpPosition = -8.33;

    public static final double kClawDownPosition = .37;
    public static final double kClawL2Position = 3.8;
    public static final double kClawL3Position = 4.6;
    public static final double kClawL1Position = -1.48;

    public static final double kMinElevatorPosForIntakeUp = 3.7;

    public static final int kMaxClawMotorCurrent = 0;

    public static enum ELEVATOR_POSITION {
      K_L0,
      K_L1,
      K_L2,
      K_L3,
      K_EXIT
    }

    public static enum INTAKE_POSITION {
      K_GND,
      K_STW,
      K_L1A,
      K_L1,
      K_EXIt
    }
  }
}
