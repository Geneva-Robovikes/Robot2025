// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  /* Initialize swerve modules */
  private final SwerveModule frontRight = new SwerveModule(1, 3, false, true, 2, "Front Right");
  private final SwerveModule backRight = new SwerveModule(4, 6, false, true, 5, "Back Right");
  private final SwerveModule backLeft = new SwerveModule(7, 9, false, true, 8, "Back Left");
  private final SwerveModule frontLeft = new SwerveModule(10, 12, false, true, 11, "Back Right");

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.ModuleConstants.kDriveKinematics, getRotation2d(), 
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(), 
      backRight.getPosition()
    }, 

    new Pose2d(0, 0, new Rotation2d())
  );
  
  private ADIS16448_IMU gyro = new ADIS16448_IMU();

  public SwerveSubsystem() {
    /* Give the gyro time to initialize, then zero it. */
    new Thread(() -> {
      try {
        Thread.sleep(1000);
      } catch (Exception e) {}

      zeroHeading();
    });
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    /* I have no clue. */
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(-gyro.getGyroAngleZ() / 57.295779513);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    /* Reduces the wheel speeds proportionally. If the wheels are going 4,5,6,7 m/s and the max is 4 m/s,
     * the speeds are changed to 1,2,3,4 m/s.
     */
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.ModuleConstants.kMaxSpeedMetersPerSecond);

    /* Set desired states of all modules. */
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /* AUTO FUNCTIONS */

  /* Gets the states of the modules (the speed and rotation), and returns it all */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      frontLeft.getModuleState(), frontRight.getModuleState(),
      backLeft.getModuleState(), backRight.getModuleState()
    };

    return states;
  }

  /* Get the position of the robot */
  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /* Set the position of the robot */
  private void resetPose(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(), 
        backRight.getPosition()
      }, pose
    );
  }

  /* Returns ChassisSpeeds of the current states of the modules */
  private ChassisSpeeds getRobotChassisSpeeds() {
    return Constants.ModuleConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);

    setModuleStates(targetStates);
  }
}