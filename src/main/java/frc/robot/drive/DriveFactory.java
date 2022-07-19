// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ChassisType;

public class DriveFactory {
  /**
   * Create a drive based on the specified controller type
   *
   * @param motorControllerType is one of the types defined in {@link
   *     frc.robot.Constants.ChassisType}
   * @param navx is the NavX IMU
   * @returns an instance of {@link frc.robot.factories.DriveInterface}
   */
  public DriveInterface createDrive(int motorControllerType, AHRS navx) {
    DriveInterface drive;

    switch (motorControllerType) {
      case ChassisType.kCTRE:
        drive = new CTREDrive();
        SmartDashboard.putString("Selected Drive", "CTRE");
        break;
      case ChassisType.kREV:
        drive = new RevDrive();
        SmartDashboard.putString("Selected Drive", "Rev");
        break;
      case ChassisType.kHybrid:
        drive = new HybridDrive();
        SmartDashboard.putString("Selected Drive", "Hybrid");
        break;
      case ChassisType.kRomi:
        drive = new RomiDrive();
        SmartDashboard.putString("Selected Drive", "Romi");
        break;
      case ChassisType.kSwerve:
        drive = new SwerveDrive(navx);
        SmartDashboard.putString("Selected Drive", "Swerve");
        break;
      default:
        drive = new RevDrive();
        SmartDashboard.putString("Selected Drive", "Default");
        break;
    }
    return drive;
  }
}
