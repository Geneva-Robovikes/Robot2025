// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants.INTAKE_POSITION;
import frc.robot.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(13);
  private final TalonFX intakePivotMotor = new TalonFX(16);

  private final PIDController intakePidController = new PIDController(
    TunerConstants.kIntakePIDpValue, 
    TunerConstants.kIntakePIDiValue,
    TunerConstants.kIntakePIDdValue);

  private INTAKE_POSITION position = INTAKE_POSITION.K_EXIt;

  public IntakeSubsystem() {
    intakePivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakePivotMotorSpeed(double speed) {
    intakePivotMotor.set(speed);
  }

  public void setPosition(INTAKE_POSITION position) {
    this.position = position;
  }

  private double getPosition() {
    return intakePivotMotor.getPosition().getValueAsDouble();
  }


  public void setVoltage(double voltage) {
    intakePivotMotor.setVoltage(voltage);
  }

  public boolean getFinished() {
    return intakePidController.calculate(getPosition(), Constants.MechanismConstants.kIntakePivotMotorDownPosition) < 0.16;
  }

  @Override
  public void periodic() {
    if (position == INTAKE_POSITION.K_STW) {
      setVoltage(
        intakePidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kIntakePivotMotorUpPosition));

      SmartDashboard.putNumber("Intake PID:", intakePidController.calculate(getPosition(), Constants.MechanismConstants.kIntakePivotMotorUpPosition));
      SmartDashboard.putBoolean("Intake Stowed", true);
    } 
    else if (position == INTAKE_POSITION.K_GND) {
      setVoltage(
        MathUtil.clamp(intakePidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kIntakePivotMotorDownPosition), -.7, .7));

        SmartDashboard.putNumber("Intake PID:", intakePidController.calculate(getPosition(), Constants.MechanismConstants.kIntakePivotMotorDownPosition));
        SmartDashboard.putBoolean("Intake Ground", true);
    } else if (position == INTAKE_POSITION.K_L1) {
      setVoltage(
        intakePidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kIntakeL1MotorUpPosition));
    } else if (position == INTAKE_POSITION.K_L1A) {
      setVoltage(
        intakePidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kIntakeL1AutoMotorUpPosition));
    }
    else if (position == INTAKE_POSITION.K_EXIt) {
      setVoltage(0);

      SmartDashboard.putBoolean("Intake Stowed", false);
      SmartDashboard.putBoolean("Intake Ground", false);
    }

    SmartDashboard.putNumber("Intake Position:", getPosition());
  }
}
