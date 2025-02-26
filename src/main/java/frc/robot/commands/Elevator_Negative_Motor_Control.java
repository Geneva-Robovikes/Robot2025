// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */


package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator_Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class Elevator_Negative_Motor_Control extends Command {
  private final Elevator_Subsystem sohum_Elevator_Subsystem;
  /** Example static factory for an autonomous command. */
  public Elevator_Negative_Motor_Control(Elevator_Subsystem sohum_Elevator_Subsystem) {

    this.sohum_Elevator_Subsystem = sohum_Elevator_Subsystem;
    addRequirements(sohum_Elevator_Subsystem);
  }
  
  
  // initializes the operation?
  @Override
  public void initialize() {}
  
  // called when something needs to happen?
  @Override
  public void execute() {
    sohum_Elevator_Subsystem.Set_Motor_Speed(-0.5);

  }
  // called when the trigger is stopped
  @Override
  public void end(boolean interrupted) {
    sohum_Elevator_Subsystem.Set_Motor_Speed(0);
  
  // called when all of the operations are done running in this file?
  }
  @Override
  public boolean isFinished() {
    return false;
  }

    
  }
