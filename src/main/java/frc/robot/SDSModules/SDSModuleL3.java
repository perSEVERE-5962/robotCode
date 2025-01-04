// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SDSModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SDSModuleL3 extends SDSModuleBase {
  private final double BACK_RIGHT_OFFSET = 0.246582; 
  private final double BACK_LEFT_OFFSET = 0.173096; 
  private final double FRONT_RIGHT_OFFSET = 0.342529; 
  private final double FRONT_LEFT_OFFSET = 0.376221; 


  private final double kPTurning = 0.34;
  private final double kITurning = 0.0; 
  private final double kDTurning = 0.0; 
  
  private final double kDriveMotorGearRatio = 1 / 6.12; 
  private final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);
  private final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  private final double kDriveEncoderRot2Inch = kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
  private final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  private final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
  private final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
  private final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(16.6);
  //private final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4.0;

  public double getPTurning() {
    return kPTurning;
  } 

  public double getITurning() {
    return kITurning;
  } 

  public double getDTurning() {
    return kDTurning;
  }

  public double getDriveMotorGearRatio() {
    return kDriveMotorGearRatio;
  }

  public double getBackRightDriveAbsoluteEncoderOffsetDeg(){
    return Rotation2d.fromRotations(BACK_RIGHT_OFFSET).getDegrees() + 180.0;
  }

  public double getBackLeftDriveAbsoluteEncoderOffsetDeg(){
    return Rotation2d.fromRotations(BACK_LEFT_OFFSET).getDegrees() + 180.0;
  }

  public double getFrontRightDriveAbsoluteEncoderOffsetDeg(){
    return Rotation2d.fromRotations(FRONT_RIGHT_OFFSET).getDegrees() + 180.0;
  }

  public double getFrontLeftDriveAbsoluteEncoderOffsetDeg(){
    return Rotation2d.fromRotations(FRONT_LEFT_OFFSET).getDegrees() + 180.0;
  }

  public double getBackRightDriveAbsoluteEncoderOffsetRad(){
    return Math.toRadians(getBackRightDriveAbsoluteEncoderOffsetDeg());
  }

  public double getBackLeftDriveAbsoluteEncoderOffsetRad(){
    return Math.toRadians(getBackLeftDriveAbsoluteEncoderOffsetDeg());
  }

  public double getFrontRightDriveAbsoluteEncoderOffsetRad(){
    return Math.toRadians(getFrontRightDriveAbsoluteEncoderOffsetDeg());
  }

  public double getFrontLeftDriveAbsoluteEncoderOffsetRad(){
    return Math.toRadians(getFrontLeftDriveAbsoluteEncoderOffsetDeg());
  }

  public double getDriveEncoderRot2Inch() {
    return kDriveEncoderRot2Inch;
  }

  public double getDriveEncoderRot2Meter() {
    return kDriveEncoderRot2Meter;
  }
  
  public double getDriveEncoderRPM2MeterPerSec(){
    return kDriveEncoderRPM2MeterPerSec;
  }

  
  public double getTurningEncoderRot2Rad(){
    return kTurningEncoderRot2Rad;
  }

  public double getTurningEncoderRPM2RadPerSec(){
    return kTurningEncoderRPM2RadPerSec;
  }

  public double getPhysicalMaxSpeedMetersPerSecond(){
    return kPhysicalMaxSpeedMetersPerSecond;
  }

}
