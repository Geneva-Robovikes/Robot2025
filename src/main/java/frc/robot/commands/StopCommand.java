package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class StopCommand extends Command  {
    private final SwerveSubsystem swerveSubsystem;

    public StopCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    public void execute(){
      swerveSubsystem.stopModules();
    }
}