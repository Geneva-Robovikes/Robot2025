package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignmentCommand extends Command {

  private final VisionSubsystem subsystem;
  

  public VisionAlignmentCommand(VisionSubsystem s) {
   subsystem = s;

   addRequirements(subsystem);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> optTarget = subsystem.getTarget();

    int targetID;

    if (optTarget.isPresent()) {
      PhotonTrackedTarget target = optTarget.get();

      for(int x = 0; x < Constants.VisionConstants.kReefAprilTags.length; x++) {
        if (target.getFiducialId() == Constants.VisionConstants.kReefAprilTags[x]) {
          targetID = target.getFiducialId();
          
          SmartDashboard.putBoolean("Target?", true);
          SmartDashboard.putNumber("Vision Target ID", targetID);
        }
      }
    }

    SmartDashboard.putBoolean("Target?", false);
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
