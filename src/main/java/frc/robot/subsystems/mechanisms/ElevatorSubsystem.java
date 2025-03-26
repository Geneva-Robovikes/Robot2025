// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0);

  private final TrapezoidProfile profile = new TrapezoidProfile(new Constraints(.5, .1));

  private ELEVATOR_POSITION position = ELEVATOR_POSITION.K_EXIT;

  public ElevatorSubsystem() {}

  public void setElevatorMotorSpeed(double speed) {
    elevatorMotor.set(speed);
    neoVortexOne.set(speed);
    neoVortexTwo.set(speed);
  }

  public void setPosition(ELEVATOR_POSITION position) {
    this.position = position;
  }

  public void setVoltage(double voltage) {
    elevatorMotor.set(voltage);
    neoVortexOne.set(voltage);
    neoVortexTwo.set(voltage);
  }

  public void setSpeed(double voltage) {
    SmartDashboard.putNumber("Requested Speed:", voltage);
  
    elevatorMotor.set(voltage);
    neoVortexOne.set(voltage);
    neoVortexTwo.set(voltage);
  }

  private double getPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean atSetpoint(ELEVATOR_POSITION goal) {
    switch(goal) {
      case K_L0:
        return elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawDownPosition) < 0.2;
      case K_L1:
        return elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL1Position) < 0.2;
      case K_L2:
        return elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position) < 0.2;
      default:
        return false;
    }
  }

  @Override
  public void periodic() {
    if (position == ELEVATOR_POSITION.K_L0) {
      double pid = elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawDownPosition);
      setSpeed(pid);

      SmartDashboard.putNumber("Elevator PID L0:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawDownPosition));
      SmartDashboard.putBoolean("Elevator L0", true);
    } 
    else if (position == ELEVATOR_POSITION.K_L1) {
      double pid = elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL1Position);
      setSpeed(pid);

      SmartDashboard.putNumber("Elevator PID L1:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL1Position));
      SmartDashboard.putBoolean("Elevator L1", true);
    }
    else if (position == ELEVATOR_POSITION.K_L2) {
      double pid = elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position);
      setSpeed(pid);

      SmartDashboard.putNumber("Elevator PID L2:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position));
      SmartDashboard.putBoolean("Elevator L2", true);
    }
    else if (position == ELEVATOR_POSITION.K_L3) {
      setSpeed(
        elevatorPidController.calculate(
        getPosition(), 
        Constants.MechanismConstants.kClawL3Position));

      SmartDashboard.putNumber("Elevator PID L3:", elevatorPidController.calculate(getPosition(), Constants.MechanismConstants.kClawL2Position));
      SmartDashboard.putBoolean("Elevator L3", true);
    } else if (position == ELEVATOR_POSITION.K_EXIT) {
          
      setSpeed(0);

      SmartDashboard.putBoolean("Elevator L0:", false);
      SmartDashboard.putBoolean("Elevator L1:", false);
      SmartDashboard.putBoolean("Elevator L2:", false);
      SmartDashboard.putBoolean("Elevator L3:", false);

      SmartDashboard.putNumber("Elevator Encoder Position: ", getPosition());
    }
  }
}
