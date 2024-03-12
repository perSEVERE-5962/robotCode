// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import frc.robot.Constants;

public class RockwellUltrasonic extends UltrasonicAnalog {
  public RockwellUltrasonic(int kintakeAnalogChannel) {
    super(kintakeAnalogChannel);
  }

  public double getRange() {
    double range = ultrasonic.getVoltage() * valueToInches;
    // handle case where ultrasonic isn't working properly
    if (range < 1.0) {
      range = Constants.UltrasonicConstants.kNotDetectedRange;
    }
    return range;
  }

}