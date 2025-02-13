// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  public LEDSubsystem() {
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(100);

    led.setLength(ledBuffer.getLength());
    
    led.start();
  }

  public void setColor(LEDPattern pattern) {
    pattern.atBrightness(Percent.of(30));
    pattern.applyTo(ledBuffer);
    
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
  }
}
