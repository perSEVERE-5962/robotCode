// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicAnalog {
  protected AnalogInput ultrasonic;
  protected double valueToInches = 2.3; // rockwell default

  public UltrasonicAnalog(int kintakeAnalogChannel) {
    ultrasonic = new AnalogInput(kintakeAnalogChannel);
  }

  public UltrasonicAnalog(int kintakeAnalogChannel, double valueToInches) {
    ultrasonic = new AnalogInput(kintakeAnalogChannel);
    this.valueToInches = valueToInches;
  }

  public double getRange() {
    return ultrasonic.getVoltage() * valueToInches;
  }

  public double getVoltage() {
    return ultrasonic.getVoltage();
  }

  public boolean isEnabled() {
    return true;
  }

}