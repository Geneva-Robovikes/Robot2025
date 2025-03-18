// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants.ELEVATOR_POSITION;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorStateCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  
  private ELEVATOR_POSITION position;

  public ElevatorStateCommand(ElevatorSubsystem elevatorSubsystem, ELEVATOR_POSITION elevatorPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = elevatorPosition;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setPosition(position);
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
