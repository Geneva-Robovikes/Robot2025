// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.Constants;

public class ClawL1Preset extends Command {
  private final ElevatorSubsystem elevatorSubsystem;

  //private final PIDController intakePositionPID;

  private boolean done;

  public ClawL1Preset(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    //intakePositionPID = new PIDController(TunerConstants.kElevatorPIDpValue, TunerConstants.kElevatorPIDiValue, TunerConstants.kElevatorPIDdValue);

    done = false;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //elevatorSubsystem.setElevatorMotorSpeed(MathUtil.clamp((intakePositionPID.calculate(elevatorSubsystem.getElevatorMotorPosition(), Constants.MechanismConstants.kClawDownPosition)), -1, 1));

    elevatorSubsystem.setElevatorMotorSpeed(.22);

    double diff = Math.abs(elevatorSubsystem.getElevatorMotorPosition()) - Math.abs(Constants.MechanismConstants.kClawL1Position);

    if (diff < 0) {
      System.out.println("h");
      done = true;
    } else {
      done = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
