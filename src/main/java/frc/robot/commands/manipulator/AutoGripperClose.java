// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import frc.robot.Constants;
import frc.robot.sensors.UltrasonicAnalog;

public class AutoGripperClose extends GripperClose {

  private UltrasonicAnalog sensor;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensor = new UltrasonicAnalog(Constants.GripperConstants.kSensorChannel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (sensor.getRange() > Constants.GripperConstants.kMinRange
        && sensor.getRange() < Constants.GripperConstants.kMaxRange);
  }
}
