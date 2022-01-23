// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import frc.robot.Constants.MotorControllerType;

/**
 * Create a drive based on the specified controller type
 *
 * @param motorControllerType is one of the types defined in {@link
 *     frc.robot.Constants.MotorControllerType}
 * @returns an instance of {@link frc.robot.factories.DriveInterface}
 */
public class DriveFactory {
  /**
   * create the drive system based on the controller type
   *
   * <p>starting with the 2022 season we are switching from CTRE (TalonSRX & VictorSPX) to REV
   * (Spark Max) controllers
   *
   * @return the drive system to use
   */
  public DriveInterface createDrive() {
    return createDrive(MotorControllerType.kREV);
  }

  public DriveInterface createDrive(int motorControllerType) {
    DriveInterface drive;
    switch (motorControllerType) {
      case MotorControllerType.kCTRE:
        drive = new CTREDrive();
        break;
      case MotorControllerType.kREV:
        drive = new RevDrive();
        break;
      default:
        drive = new RevDrive(); // default to Rev starting in 2022
        break;
    }
    return drive;
  }
}
