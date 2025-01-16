package frc.robot.commands;

import java.util.Optional;
import java.util.stream.IntStream;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignmentCommand extends Command {

  private final VisionSubsystem subsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController controller;

  public VisionAlignmentCommand(VisionSubsystem s, SwerveSubsystem ss) {
    subsystem = s;
    swerveSubsystem = ss;
    controller = new PIDController(Constants.ModuleConstants.kDrivePIDkValue, 0, 0);

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

    if (optTarget.isPresent()) {
      PhotonTrackedTarget target = optTarget.get();

      targetID = target.getFiducialId();
      contains = IntStream.of(Constants.VisionConstants.kReefAprilTags).anyMatch(x -> x == targetID);

      if(contains) {
        chassisSpeeds = new ChassisSpeeds(0, (Constants.VisionConstants.kMaxVisionAlignmentSpeed * controller.calculate(Constants.VisionConstants.kReefYawOffset, target.getYaw())), 0);
        moduleStates = Constants.ModuleConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
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
