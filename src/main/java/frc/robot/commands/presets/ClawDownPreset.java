// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.TunerConstants;


public class ClawDownPreset extends Command {
  private final ElevatorSubsystem elevatorSubsystem;

  private final PIDController intakePositionPID;

  private boolean done;

  public ClawDownPreset(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    intakePositionPID = new PIDController(TunerConstants.kElevatorPIDpValue, TunerConstants.kElevatorPIDiValue, TunerConstants.kElevatorPIDdValue);

    done = false;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //elevatorSubsystem.setElevatorMotorSpeed(MathUtil.clamp((intakePositionPID.calculate(elevatorSubsystem.getElevatorMotorPosition(), Constants.MechanismConstants.kClawDownPosition)), -1, 1));

    elevatorSubsystem.setElevatorMotorSpeed(-.19);

    double diff = Math.abs(elevatorSubsystem.getElevatorMotorPosition()) - Math.abs(Constants.MechanismConstants.kClawDownPosition);

    if (diff > 0) {
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
