// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.MotorSubsystem;
import frc.robot.Constants;
import frc.robot.TunerConstants;


public class IntakeDownPreset extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final MotorSubsystem motorSubsystem;

  private final PIDController intakePositionPID;

  private boolean done;

  public IntakeDownPreset(IntakeSubsystem intakeSubsystem, MotorSubsystem motorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.motorSubsystem = motorSubsystem;

    intakePositionPID = new PIDController(TunerConstants.kIntakePIDpValue, TunerConstants.kIntakePIDiValue, TunerConstants.kIntakePIDdValue);

    done = false;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    motorSubsystem.getElevatorMotorPosition();
    intakeSubsystem.setIntakePivotMotorSpeed(MathUtil.clamp((intakePositionPID.calculate(intakeSubsystem.getIntakeMotorPosition(), Constants.MechanismConstants.kIntakePivotMotorDownPosition)), -.13, .13));

    double diff = intakeSubsystem.getIntakeMotorPosition() - Constants.MechanismConstants.kIntakePivotMotorDownPosition;

    if (diff > 0) {
      done = true;
    } else {
      done = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePivotMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
