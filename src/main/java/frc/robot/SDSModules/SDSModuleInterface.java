// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SDSModules;

/** Add your docs here. */
public interface SDSModuleInterface {

  double getWheelDiameterInches();
  
  double getDriveEncoderRot2Inch();

  double getDriveMotorGearRatio();

  double getDriveEncoderRot2Meter();

  double getDriveEncoderRPM2MeterPerSec();

  double getTurningEncoderRot2Rad();

  double getTurningEncoderRPM2RadPerSec();

  double getBackRightDriveAbsoluteEncoderOffsetDeg();

  double getBackLeftDriveAbsoluteEncoderOffsetDeg();

  double getFrontRightDriveAbsoluteEncoderOffsetDeg();

  double getFrontLeftDriveAbsoluteEncoderOffsetDeg();

  double getBackRightDriveAbsoluteEncoderOffsetRad();

  double getBackLeftDriveAbsoluteEncoderOffsetRad();

  double getFrontRightDriveAbsoluteEncoderOffsetRad();

  double getFrontLeftDriveAbsoluteEncoderOffsetRad();

  double getPTurning();

  double getITurning();

  double getDTurning();

  double getPhysicalMaxSpeedMetersPerSecond();
}
