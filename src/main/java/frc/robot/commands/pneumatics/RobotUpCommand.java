// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.PneumaticSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotUpCommand extends Command {
  private final PneumaticSubsystem pneumaticSubsystem;

  public RobotUpCommand(PneumaticSubsystem pneumaticSubsystem) {
    this.pneumaticSubsystem = pneumaticSubsystem;

    addRequirements(pneumaticSubsystem);
  }

  @Override
  public void initialize() {
    pneumaticSubsystem.toggleOne();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
