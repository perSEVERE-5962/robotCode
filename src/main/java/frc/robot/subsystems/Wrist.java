// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

public class Wrist extends Actuator {
  private static Wrist instance;

  private Wrist() {
    super(
        Constants.WristConstants.kWristID,
        Constants.WristConstants.kP,
        Constants.WristConstants.kI,
        Constants.WristConstants.kD,
        Constants.WristConstants.kMinOutput,
        Constants.WristConstants.kMaxOutput,
        Constants.WristConstants.kFF,
        Constants.WristConstants.kIz,
        Constants.WristConstants.kUpperSoftLimit);
  }

  @Override
  public void periodic() {
  }

  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }
}
