// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(13);
  private final TalonFX intakePivotMotor = new TalonFX(16);

  public IntakeSubsystem() {}

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakePivotMotorSpeed(double speed) {
    intakePivotMotor.set(speed);
  }

  public double getClawMotorCurrent() {
    SmartDashboard.putNumber("Talon Current", intakeMotor.getMotorVoltage().getValueAsDouble());
    return intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
