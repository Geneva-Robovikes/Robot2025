package frc.robot.commands;

import java.util.Optional;
import java.util.stream.IntStream;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignmentCommand extends Command {

  private final VisionSubsystem subsystem;
  private final SwerveSubsystem swerveSubsystem;

  private final PIDController yawController;
  private final PIDController distanceController;

  private boolean stop;

  public VisionAlignmentCommand(VisionSubsystem s, SwerveSubsystem ss) {
    subsystem = s;
    swerveSubsystem = ss;

    yawController = new PIDController(.015, 0, 0);
    distanceController = new PIDController(.015, 0, 0);

    stop = false;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> optTarget = subsystem.getTarget();

    int targetID;
    boolean contains;
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] moduleStates;

    double targetRange;

    if (optTarget.isPresent()) {
      PhotonTrackedTarget target = optTarget.get();

      targetID = target.getFiducialId();
      contains = IntStream.of(Constants.VisionConstants.kReefAprilTags).anyMatch(x -> x == targetID);

      if(contains) {
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
          Constants.VisionConstants.kCameraHeight, // Measured with a tape measure, or in CAD.
          Constants.VisionConstants.kReefAprilTagHeight, // From 2024 game manual for ID 7
          Units.degreesToRadians(11.0), // Measured with a protractor, or in CAD.
          Units.degreesToRadians(target.getPitch()));

        SmartDashboard.putNumber("Range", targetRange);
        SmartDashboard.putNumber("Yaw", target.getYaw());
        SmartDashboard.putNumber("Skew", target.getSkew());

        SmartDashboard.putNumber("PID Distance", MathUtil.clamp(distanceController.calculate((Constants.VisionConstants.kReefDistanceOffset * 10), (targetRange*10)), -1, 1));
        
        /* 
        chassisSpeeds = new ChassisSpeeds((Constants.VisionConstants.kMaxVisionDistAlignmentSpeed * MathUtil.clamp(distanceController.calculate((Constants.VisionConstants.kReefDistanceOffset * 10), (targetRange*10)), -1, 1)), -(Constants.VisionConstants.kMaxVisionAlignmentSpeed * yawController.calculate(Constants.VisionConstants.kReefYawOffset, target.getYaw())), 0);
        moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        */
        double speed = yawController.calculate(Constants.VisionConstants.kReefYawOffset, target.getYaw()) * Constants.VisionConstants.kMaxVisionAlignmentSpeed;

        chassisSpeeds = new ChassisSpeeds(0, (-speed), 0);
        moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        

        swerveSubsystem.setModuleStates(moduleStates);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    
    return stop;
  }
}
