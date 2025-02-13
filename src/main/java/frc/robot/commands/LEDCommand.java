// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDCommand extends Command {
  private final LEDSubsystem subsystem;

  public LEDCommand(LEDSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    
    if(alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        subsystem.setColor(LEDPattern.solid(Color.kRed), Optional.of(30));
      } else {
        subsystem.setColor(LEDPattern.solid(Color.kBlue), Optional.of(30));
      }
    } else {
      subsystem.setColor(LEDPattern.solid(Color.kWhite), Optional.of(30));
    }
    // To make brightness default, plug Optional.empty
  }


  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
