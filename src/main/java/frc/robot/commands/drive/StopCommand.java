package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;


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