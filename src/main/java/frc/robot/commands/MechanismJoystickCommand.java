// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MechanismJoystickCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  private final LEDSubsystem ledSubsystem;

  private final CommandXboxController controller;

  public MechanismJoystickCommand(MotorSubsystem subsystem, LEDSubsystem ledSubsystem, CommandXboxController controller) {
    motorSubsystem = subsystem;
    this.ledSubsystem = ledSubsystem;
    this.controller = controller;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent) {
      ledSubsystem.flashColor(LEDPattern.solid(Color.kGreen), .3);
    }
    
    if (controller.leftTrigger().getAsBoolean()) {
      motorSubsystem.runClaw();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent;
  }
}
