// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeJoystickCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  XboxController controller;

  double axis;
  /** Creates a new IntakeJoystickCommand. */
  public IntakeJoystickCommand(IntakeSubsystem intakeSubsystem) {
    controller = new XboxController(1);
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    axis = controller.getLeftY();

    intakeSubsystem.setIntakePivotMotorSpeed(axis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
