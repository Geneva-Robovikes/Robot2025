// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@SuppressWarnings("removal")
public class MotorSubsystem extends SubsystemBase {
  private final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Units.Volts);
  private final MutDistance m_distance = new MutDistance(0, 0, Units.Meters);
  private final MutLinearVelocity m_velocity = new MutLinearVelocity(0, 0, Units.MetersPerSecond);


  /* TODO: real can id's */
  private final TalonFX elevatorMotor = new TalonFX(14);
  private final TalonFX intakeTiltMotor = new TalonFX(0);

  public MotorSubsystem() {
    intakeTiltMotor.setInverted(false);
  }
  

  @Override
  public void periodic() {}

  public void setElevatorMotorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void setTiltMotorSpeed(double speed) {
    intakeTiltMotor.set(speed);
  }
  public double getElevatorMotorPosition(){
    SmartDashboard.putNumber("elevator pos", elevatorMotor.getPosition().getValueAsDouble());
    return elevatorMotor.getPosition().getValueAsDouble();
  }



  /*************************/
  /* SYSTEM IDENTIFICATION */
  /*************************/

  /**************************/
  /* CONFIGURE ACCORDING TO */
  /*THE MOTOR YOU ARE TUNING*/
  /**************************/

  public double getDriveVoltage() {
    return elevatorMotor.get() * RobotController.getBatteryVoltage();
  }

  public double getDrivePosition() {
    return elevatorMotor.getRotorPosition().getValueAsDouble();
  }

  public double getDriveVelocity() {
    return elevatorMotor.getRotorVelocity().getValueAsDouble();
  }

  /* Set up a new SysId routine */
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),

    new SysIdRoutine.Mechanism(
      volts -> elevatorMotor.setVoltage(volts.in(Volts)),
      log -> {
        log.motor("Elevator Motor").voltage(m_appliedVoltage.mut_replace(getDriveVoltage(), Volts));
        log.motor("Elevator Motor").linearPosition(m_distance.mut_replace(getDrivePosition(), Units.Meters));
        log.motor("Elevator Motor").linearVelocity(m_velocity.mut_replace(getDriveVelocity(), Units.MetersPerSecond));
      },
      this
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
