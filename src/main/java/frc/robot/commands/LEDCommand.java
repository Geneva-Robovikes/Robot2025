// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.util.LED;
import frc.robot.subsystems.util.Vision;
import frc.robot.subsystems.mechanisms.MotorSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDCommand extends Command {
  private final LED ledSubsystem;
  private final Vision visionSubsystem;
  private final MotorSubsystem motorSubsystem;

  private LEDPattern defaultColor = LEDPattern.solid(null);

  public LEDCommand(LED ledSubsystem, Vision visionSubsystem, MotorSubsystem motorSubsystem) {
    this.ledSubsystem = ledSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.motorSubsystem = motorSubsystem;

    defaultColor = LEDPattern.solid(Color.kWhite);

  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    
    if(alliance.isPresent()) {
      if(alliance.get() == DriverStation.Alliance.Red) {
        defaultColor = LEDPattern.solid(Color.kRed);

        ledSubsystem.setColor(defaultColor);
      } else {
        defaultColor = LEDPattern.solid(Color.kBlue);

        ledSubsystem.setColor(defaultColor);
      }
    } else {
      ledSubsystem.setColor(LEDPattern.solid(Color.kWhite));
    }
  }


  @Override
  public void execute() {
    if(visionSubsystem.targetReady()) {
      ledSubsystem.setColor(LEDPattern.solid(Color.kGreen));
    } else if(visionSubsystem.targetInView()) {
      ledSubsystem.setColor(LEDPattern.solid(Color.kGreenYellow));
    } else if(motorSubsystem.getClawMotorCurrent() > Constants.MechanismConstants.kMaxClawMotorCurrent) {
      ledSubsystem.flashColor(LEDPattern.solid(Color.kGreen), .2);
    } else {
      ledSubsystem.setColor(defaultColor);
    }
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
