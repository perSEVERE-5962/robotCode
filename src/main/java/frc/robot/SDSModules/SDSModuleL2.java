// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SDSModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SDSModuleL2 extends SDSModuleBase {
  // chassis #2
  // private final double BACK_RIGHT_OFFSET = 0.473389; /*0.251221; //chassis one 12*/
  // private final double BACK_LEFT_OFFSET = 0.049805; /*0.167236;//chassis one 22 */
  // private final double FRONT_RIGHT_OFFSET = 0.024170; /* 0.346436; //chassis one 32 */
  // private final double FRONT_LEFT_OFFSET = 0.291016; /* 0.379150;//chassis one  */

  //chassis #1
  private final double BACK_RIGHT_OFFSET = 0.254883;
  private final double BACK_LEFT_OFFSET = 0.163574; 
  private final double FRONT_RIGHT_OFFSET = 0.330566;
  private final double FRONT_LEFT_OFFSET = -0.429199; 
  

  private final double kPTurning = 0.34;
  private final double kITurning = 0.0; 
  private final double kDTurning = 0.0; 
  
  private final double kDriveMotorGearRatio = 1.0 / 6.75; 
  private final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);
  private final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  private final double kDriveEncoderRot2Inch = kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
  private final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  private final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
  private final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
  private final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1);  
  private final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4.0;

  protected double getBackRightOffset() {
    return BACK_RIGHT_OFFSET;
  }

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

  public double getTeleDriveMaxSpeedMetersPerSecond() {
    return kTeleDriveMaxSpeedMetersPerSecond;
  }
}
