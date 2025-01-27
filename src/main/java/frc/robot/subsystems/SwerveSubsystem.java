// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  /* Initialize swerve modules */
  private final SwerveModule frontLeft = new SwerveModule(1, 3, false, true, 2, "Front Right");
  private final SwerveModule backLeft = new SwerveModule(4, 6, false, true, 5, "Back Right");
  private final SwerveModule backRight = new SwerveModule(7, 9, false, true, 8, "Back Left");
  private final SwerveModule frontRight = new SwerveModule(10, 12, false, true, 11, "Back Right");
  
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();

  private final Field2d field = new Field2d();

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.ModuleConstants.kDriveKinematics, getRotation2d(), 
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(), 
      backRight.getPosition()
    }, 

    new Pose2d(0, 0, new Rotation2d())
  );
  

  public SwerveSubsystem() {
    SmartDashboard.putData("Field", field);

    gyro.calibrate();

    /* AUTO */
    RobotConfig config;
    
    try{
      config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            //3k for position, .15 for velociry
              new PIDConstants(3, 2.7, 0), // Translation PID constants
              new PIDConstants(3, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(-gyro.getGyroAngleZ() / 57.295779513);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", -gyro.getGyroAngleZ() / 57.295779513);

    odometry.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()
    });

    field.setRobotPose(odometry.getPoseMeters());
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
