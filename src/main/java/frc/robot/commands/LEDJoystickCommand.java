// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class LEDJoystickCommand extends Command {

  private final LEDSubsystem subsystem;

  public LEDJoystickCommand(LEDSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    subsystem.setPattern();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
