package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignmentCommand extends Command {

  private final VisionSubsystem subsystem;
  private final SwerveSubsystem swerveSubsystem;

  public VisionAlignmentCommand(VisionSubsystem s, SwerveSubsystem ss) {
    subsystem = s;
    swerveSubsystem = ss;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> optTarget = subsystem.getTarget();

    int targetID;
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] moduleStates;

    if (optTarget.isPresent()) {
      PhotonTrackedTarget target = optTarget.get();

      targetID = target.getFiducialId();

      for(int x = 0; x < Constants.VisionConstants.kReefAprilTags.length; x++) {
        if(targetID == Constants.VisionConstants.kReefAprilTags[x]) {
          if (target.getYaw() > Constants.VisionConstants.kReefYawOffset) {
            chassisSpeeds = new ChassisSpeeds(0, Constants.VisionConstants.kMaxVisionAlignmentSpeed, 0);
            moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
          } else if (target.getYaw() < Constants.VisionConstants.kReefYawOffset) {
            chassisSpeeds = new ChassisSpeeds(0, -Constants.VisionConstants.kMaxVisionAlignmentSpeed, 0);
            moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
          }
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
