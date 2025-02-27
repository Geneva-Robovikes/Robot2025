// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.MotorSubsystem;
import frc.robot.util.Easings;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MechanismJoystickCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final XboxController controller;
  private final Easings easings;

  public MechanismJoystickCommand(MotorSubsystem subsystem, LEDSubsystem ledSubsystem, XboxController controller) {
    motorSubsystem = subsystem;
    this.ledSubsystem = ledSubsystem;
    this.controller = controller;
    easings = new Easings();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent) {
      ledSubsystem.flashColor(LEDPattern.solid(Color.kGreen), .3);
    }

    // INTAKE SYSTEM
      // Intake system codes will be placed under Intake System title
    while (controller.getRightBumperButtonPressed() == true) {
      motorSubsystem.setElevatorMotorSpeed(0.5);
    }

    // CLAW
      // Claw codes will be placed under Claw title
    while (controller.getLeftBumperButtonPressed() == true) {
      motorSubsystem.setClawMotorSpeed(0.5);
      motorSubsystem.setIntakeMotorSpeed(0.5);
    }
    // ELEVATOR 
    // ELEVATOR LEFT & TRIGGERS CONTROL (BASIC SYSTEM FOR SATURDAY)

    if (controller.getLeftTriggerAxis() > Constants.ModuleConstants.kDeadzoneMinimum && controller.getLeftTriggerAxis() < Constants.ModuleConstants.kDeadzoneMaximum) {

      double leftTriggerSpeed = easings.joystick(easings.joystick(controller.getLeftTriggerAxis()), 2); 


    } else if (controller.getRightTriggerAxis() > Constants.ModuleConstants.kDeadzoneMinimum && controller.getRightTriggerAxis() < Constants.ModuleConstants.kDeadzoneMaximum) {

      double rightTriggerSpeed = easings.joystick(easings.joystick(controller.getRightTriggerAxis()), 2) * -1;  


    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent;
  }

}
