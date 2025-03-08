// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private final TalonFX clawMotor = new TalonFX(15);

  public ClawSubsystem() {
    clawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getClawMotorCurrent() {
    SmartDashboard.putNumber("Talon Current", clawMotor.getMotorVoltage().getValueAsDouble());
    return clawMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setClawMotorSpeed(double speed) {
    clawMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
