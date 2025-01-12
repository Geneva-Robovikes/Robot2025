package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
