// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.INTAKE_POSITION;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeStateCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  
  private INTAKE_POSITION position;

  public IntakeStateCommand(IntakeSubsystem intakeSubsystem, INTAKE_POSITION intakePosition) {
    this.intakeSubsystem = intakeSubsystem;
    this.position = intakePosition;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setPosition(position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intakeSubsystem.getFinished();
  }
}
