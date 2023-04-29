// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class UltrasonicAnalog {
  private static UltrasonicAnalog instance;
  private AnalogInput ultrasonic;
  private final double valueToInches = 2.3;

  private UltrasonicAnalog() {
    ultrasonic = new AnalogInput(Constants.UltrasonicConstants.kSensor_Analog_Channel);
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

  /**
   * @return the instance
   */
  public static UltrasonicAnalog getInstance() {
    if (instance == null) {
      instance = new UltrasonicAnalog();
    }

    return instance;
  }
}
