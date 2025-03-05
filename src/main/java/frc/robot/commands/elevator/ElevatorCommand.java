// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.MotorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  
  private double maxSpeed;
  private double deadzone;

  private final XboxController controller;
  private double rightTriggerAxis;
  private double leftTriggerAxis;
  private double combinedTriggerAxis;

  /** Creates a new ElevatorUpCommand. */
  public ElevatorCommand(MotorSubsystem motorSubsystem) {
    this.motorSubsystem = motorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    controller = new XboxController(0);
  
    maxSpeed = Constants.MechanismConstants.kElevatorMotorMaximumSpeed;
    deadzone = Constants.MechanismConstants.kElevatorMotorDeadzone;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightTriggerAxis = controller.getRightTriggerAxis();
    leftTriggerAxis = controller.getLeftTriggerAxis();

    if (rightTriggerAxis < .5) {
      rightTriggerAxis = .5;
    }
    if (leftTriggerAxis < .5) {
      leftTriggerAxis = .5;
    }
    
    rightTriggerAxis = (rightTriggerAxis - .5) * 2;
    leftTriggerAxis = (leftTriggerAxis - .5) * 2;

    combinedTriggerAxis = rightTriggerAxis - leftTriggerAxis;

    // rightTriggerAxis = easings.joystick(rightTriggerAxis);
    // leftTriggerAxis = easings.joystick(leftTriggerAxis);
    combinedTriggerAxis = combinedTriggerAxis * maxSpeed;
    if (-deadzone <= combinedTriggerAxis && combinedTriggerAxis < deadzone) {
      leftTriggerAxis = 0;
    }

    // System.out.println(combinedTriggerAxis);
    motorSubsystem.setElevatorMotorSpeed(combinedTriggerAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.setElevatorMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
