// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.TunerConstants;


public class IntakeDownPreset extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PIDController intakePositionPID;

  public IntakeDownPreset(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    intakePositionPID = new PIDController(TunerConstants.kIntakePIDpValue, TunerConstants.kIntakePIDiValue, TunerConstants.kIntakePIDdValue);

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakePivotMotorSpeed(MathUtil.clamp(intakePositionPID.calculate(intakeSubsystem.getIntakeMotorPosition(), Constants.MechanismConstants.kIntakePivotMotorDownPosition), -.1, .1));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
