// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LED_Button_Command extends Command {
  private final LEDSubsystem ledSubsystem;
  
  private final LEDPattern rainbow;
  private final AddressableLED led;

  public LED_Button_Command(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    
    led = new AddressableLED(9);
    rainbow = LEDPattern.rainbow(100, 128);
  }   

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    ledSubsystem.flashColor(rainbow, 4);
  }


  @Override
  public void end(boolean interrupted) {
    led.stop();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
