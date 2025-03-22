// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntakeOutCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final Timer timer = new Timer();

  private boolean stop = false;

  public AutoIntakeOutCommand(IntakeSubsystem subsystem) {
    this.intakeSubsystem = subsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
    System.out.println("initialized");
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(.4)) {
      stop = true;
      System.out.println("timer elapsed");
    }

    System.out.println("executing");

    intakeSubsystem.setIntakeMotorSpeed(.18);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    intakeSubsystem.setIntakeMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
