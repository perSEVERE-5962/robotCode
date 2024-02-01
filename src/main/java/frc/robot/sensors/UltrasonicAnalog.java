// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicAnalog {
  private AnalogInput ultrasonic;
  private final double valueToInches = 2.3;

  public UltrasonicAnalog(int kintakeAnalogChannel,int kIntake_PCM_Channel ) {
    ultrasonic = new AnalogInput(kintakeAnalogChannel);
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