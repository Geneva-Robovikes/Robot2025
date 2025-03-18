// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import frc.robot.Constants.MechanismConstants.ELEVATOR_POSITION;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(14);
  private final SparkMax neoVortexOne = new SparkMax(19, MotorType.kBrushless);
  private final SparkMax neoVortexTwo = new SparkMax(20, MotorType.kBrushless);

  private final PIDController elevatorPidController = new PIDController(
    TunerConstants.kElevatorPIDpValue, 
    TunerConstants.kElevatorPIDiValue,
    TunerConstants.kElevatorPIDdValue);

  private ELEVATOR_POSITION position = ELEVATOR_POSITION.K_L0;

  public ElevatorSubsystem() {}

  public void setElevatorMotorSpeed(double speed) {
    elevatorMotor.set(speed);
    neoVortexOne.set(speed);
    neoVortexTwo.set(speed);
  }

  public void setPosition(ELEVATOR_POSITION position) {
    this.position = position;
  }

  private void setVoltages(double voltage) {
    elevatorMotor.setVoltage(voltage);
    neoVortexOne.setVoltage(voltage);
    neoVortexTwo.setVoltage(voltage);
  }

  private double getPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    if (position == ELEVATOR_POSITION.K_L0) {
      setVoltages(
        elevatorPidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kClawDownPosition));

      SmartDashboard.putNumber("Elevator PID:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawDownPosition));
      SmartDashboard.putBoolean("Elevator L0", true);
    } 
    else if (position == ELEVATOR_POSITION.K_L1) {
      setVoltages(
        elevatorPidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kClawL1Position));

        SmartDashboard.putNumber("Elevator PID:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL1Position));
        SmartDashboard.putBoolean("Elevator L1", true);
    }
    else if (position == ELEVATOR_POSITION.K_L2) {
      setVoltages(
        elevatorPidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kClawL2Position));

        SmartDashboard.putNumber("Elevator PID:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position));
        SmartDashboard.putBoolean("Elevator L2", true);
    }
    else if (position == ELEVATOR_POSITION.K_L3) {
      setVoltages(
        elevatorPidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kClawL2Position));

      SmartDashboard.putNumber("Elevator PID:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position));
      SmartDashboard.putBoolean("Elevator L3", true);
    } 
    else {
      setVoltages(0);

      SmartDashboard.putBoolean("Elevator L0:", false);
      SmartDashboard.putBoolean("Elevator L1:", false);
      SmartDashboard.putBoolean("Elevator L2:", false);
      SmartDashboard.putBoolean("Elevator L3:", false);
    }

    SmartDashboard.putNumber("Elevator Encoder Position: ", getPosition());
  }
}
