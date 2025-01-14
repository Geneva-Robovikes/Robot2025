package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignmentCommand extends Command {

  private final VisionSubsystem subsystem;

  private boolean toggle;

  public VisionAlignmentCommand(VisionSubsystem s) {
    subsystem = s;
    toggle = false;
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

      targetID = target.getFiducialId();

      for(int x = 0; x < Constants.VisionConstants.kReefAprilTags.length; x++) {
        if(targetID == Constants.VisionConstants.kReefAprilTags[x]) {
          SmartDashboard.putBoolean("Target?", true);
          SmartDashboard.putNumber("Vision Target ID", targetID);
          SmartDashboard.putNumber("Yaw", target.getYaw());

          /* TODO: Write code that aligns the april tag to a certain offset (in yaw) */
        }
      }
    }
    else {
      SmartDashboard.putBoolean("Target?", false);
    }

    toggle = true;
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return toggle;
  }
}
