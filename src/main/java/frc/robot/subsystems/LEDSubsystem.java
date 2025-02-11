// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  private final LEDPattern rainbow;
  private final LEDPattern scrollingRainbow;

  private final Distance ledSpacing = Meters.of(1 / 100.0);

  public LEDSubsystem() {
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(100);

    rainbow = LEDPattern.rainbow(255, 128);
    scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(40), ledSpacing);


    led.setLength(ledBuffer.getLength());
    
    led.start();
  }

  public void setPattern() {
    scrollingRainbow.applyTo(ledBuffer);
    
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
  }
}
