/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.DriveFactory;
import frc.robot.drive.DriveInterface;

public class DriveTrain extends SubsystemBase {
  private DriveInterface m_drive;
  private boolean driveTrainSet = false;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP); // NavX connected over MXP

  public DriveTrain() {}

  public void setMotorControllerType(int motorControllerType) {
    if (driveTrainSet == false) {
      DriveFactory driveFactory = new DriveFactory();
      m_drive = driveFactory.createDrive(motorControllerType, m_navx);
      m_drive.resetEncoders();
      driveTrainSet = true;
    }
  }

  public double getAverageEncoderDistance() {
    double distance = 0;
    if (m_drive != null) {
      distance = m_drive.getAverageEncoderDistance();
    }
    return distance;
  }

  @Override
  public void periodic() {
    if (m_drive != null) {
      m_drive.periodic();
    }
  }

  public void moveDistanceWithPID(double position) {
    try {
      m_drive.moveDistanceWithPID(position);
    } catch (Exception e) {
      m_drive.stopDrive();
      SmartDashboard.putString("ERROR MESSAGE", e.getMessage());
    }
  }

  public DriveInterface getDriveInterface() {
    return m_drive;
  }

  // public void tankDrive(double leftAxis, double rightAxis) {
  //   m_drive.tankDrive(leftAxis, rightAxis);
  // }

  // public void arcadeDrive(double leftAxis, double rightAxis) {
  //   m_drive.arcadeDrive(leftAxis, rightAxis);
  // }

  // public void stopDrive() {
  //   m_drive.stopDrive();
  // }

  // public void resetGyro() {
  // m_gyro.resetGyro();
  // }

  // public double getGyroAngle() {
  // return m_gyro.getGyroAngle();
  // }

  // public void resetEncoders() {
  //   m_drive.resetEncoders();
  // }

  // public void setRampRate(double rate) {
  //   m_drive.setRampRate(rate);
  // }

  // public void setIdleMode(int idleMode) {
  //   m_drive.setIdleMode(idleMode);
  // }

}
