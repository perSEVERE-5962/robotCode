// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorControllerType;
import frc.robot.gyro.GyroFactory;
import frc.robot.gyro.GyroInterface;

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

  /**
   * Create a drive based on the specified controller type
   *
   * @param motorControllerType is one of the types defined in {@link
   *     frc.robot.Constants.MotorControllerType}
   * @returns an instance of {@link frc.robot.factories.DriveInterface}
   */
  public DriveInterface createDrive(int motorControllerType) {
    DriveInterface drive;
    GyroFactory gyroFactory = new GyroFactory();
    GyroInterface gyro = gyroFactory.createGyro(motorControllerType);

    switch (motorControllerType) {
      case MotorControllerType.kCTRE:
        drive = new CTREDrive(gyro);
        SmartDashboard.putString("Selected Drive", "CTRE");
        break;
      case MotorControllerType.kREV:
        drive = new RevDrive(gyro);
        SmartDashboard.putString("Selected Drive", "Rev");
        break;
      case MotorControllerType.kHybrid:
        drive = new HybridDrive(gyro);
        SmartDashboard.putString("Selected Drive", "Hybrid");
        break;
      case MotorControllerType.kRomi:
        drive = new RomiDrive(gyro);
        SmartDashboard.putString("Selected Drive", "Romi");
        break;
      case MotorControllerType.kSwerve:
        drive = new SwerveDrive(gyro);
        SmartDashboard.putString("Selected Drive", "Swerve");
        break;
      default:
        drive = new RevDrive(gyro); // default to Rev starting in 2022
        SmartDashboard.putString("Selected Drive", "Default");
        break;
    }
    return drive;
  }
}
