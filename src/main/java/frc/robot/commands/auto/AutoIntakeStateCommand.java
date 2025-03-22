// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.INTAKE_POSITION;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntakeStateCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final Timer timer = new Timer();
  
  private INTAKE_POSITION position;

  private boolean done;

  public AutoIntakeStateCommand(IntakeSubsystem intakeSubsystem, INTAKE_POSITION intakePosition) {
    this.intakeSubsystem = intakeSubsystem;
    this.position = intakePosition;

    done = false;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    done = false;
    timer.start();
    intakeSubsystem.setPosition(position);
  }

  @Override
  public void execute() {
    if (timer.hasElapsed(.6)) {
      done = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
