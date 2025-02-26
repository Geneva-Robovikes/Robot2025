// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.util.Easings;
import frc.robot.subsystems.Elevator_Subsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MechanismJoystickCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Elevator_Subsystem elevatorSubsystem;
  private final CommandXboxController controller;
  private final Easings easings;

  public MechanismJoystickCommand(MotorSubsystem subsystem, LEDSubsystem ledSubsystem, Elevator_Subsystem elevatorSubsystem, CommandXboxController controller) {
    motorSubsystem = subsystem;
    this.ledSubsystem = ledSubsystem;
    this.controller = controller;
    this.elevatorSubsystem = elevatorSubsystem;
    easings = new Easings();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent) {
      ledSubsystem.flashColor(LEDPattern.solid(Color.kGreen), .3);
    }
    
    // if (controller.leftTrigger().getAsBoolean()) {
    //   motorSubsystem.runClaw();
    // }
    
    // ELEVATOR LEFT & TRIGGERS CONTROL (BASIC SYSTEM)
   if (controller.a().getAsBoolean()) {
    elevatorSubsystem.Set_Motor_Speed(.5);
   }

    if (controller.getLeftTriggerAxis() > Constants.ModuleConstants.kDeadzoneMinimum && controller.getLeftTriggerAxis() < Constants.ModuleConstants.kDeadzoneMaximum) {

      double leftTriggerSpeed = easings.joystick(easings.joystick(controller.getLeftTriggerAxis()), 2);
      elevatorSubsystem.Set_Motor_Speed(leftTriggerSpeed);

    } else if (controller.getRightTriggerAxis() > Constants.ModuleConstants.kDeadzoneMinimum && controller.getRightTriggerAxis() < Constants.ModuleConstants.kDeadzoneMaximum) {

      double rightTriggerSpeed = easings.joystick(easings.joystick(controller.getRightTriggerAxis()), 2) * -1;
      elevatorSubsystem.Set_Motor_Speed(rightTriggerSpeed);

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.Set_Motor_Speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorSubsystem.getClawMotorCurrent() >= Constants.MechanismConstants.kMaxClawMotorCurrent;
  }

}
