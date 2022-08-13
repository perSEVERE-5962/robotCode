// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisType;

public class DriveFactory {
  /**
   * Create a drive based on the specified controller type
   *
   * @param motorControllerType is one of the types defined in {@link
   *     frc.robot.Constants.ChassisType}
   * @returns an instance of {@link frc.robot.factories.DriveInterface}
   */
  public DriveInterface createDrive(int motorControllerType) {
    DriveInterface drive;

    switch (motorControllerType) {
      case ChassisType.kREV:
        drive = new RevDrive();
        SmartDashboard.putString("Selected Drive", "Rev");
        break;
      case ChassisType.kRomi:
        drive = new RomiDrive();
        SmartDashboard.putString("Selected Drive", "Romi");
        break;
      default:
        drive = new RevDrive();
        SmartDashboard.putString("Selected Drive", "Default");
        break;
    }
    return drive;
  }
}
