// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  private final DoubleSolenoid solenoidZero;
  private final DoubleSolenoid solenoidOne;
  private final DoubleSolenoid solenoidTwo;
  private final DoubleSolenoid solenoidThree;

  //private final Compressor compressor;

  public PneumaticSubsystem() {
    solenoidZero = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    solenoidOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    solenoidTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    solenoidThree = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

    // /compressor = new Compressor(PneumaticsModuleType.REVPH);

    solenoidZero.set(DoubleSolenoid.Value.kReverse);
    solenoidOne.set(DoubleSolenoid.Value.kReverse);
    solenoidTwo.set(DoubleSolenoid.Value.kReverse);
    solenoidThree.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {}

  public void toggleZero() {
    solenoidZero.toggle();
  }

  public void toggleOne() {
    solenoidOne.toggle();
  }

  public void toggleTwo() {
    solenoidTwo.toggle();
  }

  public void toggleThree() {
    solenoidThree.toggle();
  }
}
