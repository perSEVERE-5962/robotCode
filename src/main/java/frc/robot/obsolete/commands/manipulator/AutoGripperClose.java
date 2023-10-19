// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.obsolete.commands.manipulator;

import frc.robot.Constants;
import frc.robot.sensors.UltrasonicAnalog;

public class AutoGripperClose extends GripperClose {

  private UltrasonicAnalog sensor;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensor = UltrasonicAnalog.getInstance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (sensor.getRange() > Constants.UltrasonicConstants.kMinRange
        && sensor.getRange() < Constants.UltrasonicConstants.kMaxRange);
  }
}
