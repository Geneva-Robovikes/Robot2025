// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTuner extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final Timer timer;

  private double voltage;
  private double count;

  public ElevatorTuner(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.timer = new Timer();
  }

  @Override
  public void initialize() {
    voltage = 0;
    count = 1;
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.hasElapsed((count+1)*.5)) {
      voltage += 0.01;
      count += 1;

      elevatorSubsystem.setVoltage(voltage);
      SmartDashboard.putNumber("Elevator Tuning Voltage", voltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
