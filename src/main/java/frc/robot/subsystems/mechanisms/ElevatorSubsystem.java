// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(14);

  public ElevatorSubsystem() {}

  public void setElevatorMotorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public double getElevatorMotorPosition(){
    SmartDashboard.putNumber("elevator pos", elevatorMotor.getPosition().getValueAsDouble());
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position: ", elevatorMotor.getPosition().getValueAsDouble());
  }
}
