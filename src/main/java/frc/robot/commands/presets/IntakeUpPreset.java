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


public class IntakeUpPreset extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final MotorSubsystem motorSubsystem;
  private final PIDController intakePositionPID;

  private boolean done;

  public IntakeUpPreset(IntakeSubsystem intakeSubsystem, MotorSubsystem motorSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.motorSubsystem = motorSubsystem;

    done = false;

    intakePositionPID = new PIDController(TunerConstants.kIntakePIDpValue, TunerConstants.kIntakePIDiValue, TunerConstants.kIntakePIDdValue);

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    System.out.println(intakeSubsystem.getIntakeMotorPosition());
    intakeSubsystem.setIntakePivotMotorSpeed(MathUtil.clamp((intakePositionPID.calculate(intakeSubsystem.getIntakeMotorPosition(), Constants.MechanismConstants.kIntakePivotMotorUpPosition)), -.14, .14));

    double diff = intakeSubsystem.getIntakeMotorPosition() - Constants.MechanismConstants.kIntakePivotMotorUpPosition;

    if (diff < 0 || motorSubsystem.getElevatorMotorPosition() < Constants.MechanismConstants.kMinElevatorPosForIntakeUp) {
      done = true;
    } else {
      done = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
    intakeSubsystem.setIntakePivotMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
