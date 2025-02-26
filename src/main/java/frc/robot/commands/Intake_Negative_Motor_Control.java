// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */


package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake_Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Intake_Negative_Motor_Control extends Command {
  private final Intake_Subsystem sohum_Intake_Subsystem;
  /** Example static factory for an autonomous command. */
  public Intake_Negative_Motor_Control(Intake_Subsystem sohum_Intake_Subsystem) {

    this.sohum_Intake_Subsystem = sohum_Intake_Subsystem;
    addRequirements(sohum_Intake_Subsystem);
  }
  
  
  // initializes the operation?
  @Override
  public void initialize() {}
  
  // called when something needs to happen?
  @Override
  public void execute() {
    sohum_Intake_Subsystem.set_Motor_Speed(-0.5);

  }
  // called when the trigger is stopped
  @Override
  public void end(boolean interrupted) {
    sohum_Intake_Subsystem.set_Motor_Speed(0);
  
  // called when all of the operations are done running in this file?
  }
  @Override
  public boolean isFinished() {
    return false;
  }

    
  }
