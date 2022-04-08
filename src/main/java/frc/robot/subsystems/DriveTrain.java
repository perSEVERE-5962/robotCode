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

  private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
  private DriveInterface m_drive;
  private boolean driveTrainSet = false;

  public AHRS getGyro() {
    return m_ahrs;
  }

  public DriveTrain() {
    m_ahrs.reset();
  }

  public void setMotorControllerType(int motorControllerType) {
    if (driveTrainSet == false) {
      DriveFactory driveFactory = new DriveFactory();
      m_drive = driveFactory.createDrive(motorControllerType);
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
    // This method will be called once per scheduler run

  }

  public void tankDrive(double leftAxis, double rightAxis) {
    m_drive.tankDrive(leftAxis, rightAxis);
  }

  public void arcadeDrive(double leftAxis, double rightAxis) {
    m_drive.arcadeDrive(leftAxis, rightAxis);
  }

  public void moveDistanceWithPID(double position) {
    try {
      m_drive.moveDistanceWithPID(position);
    } catch (Exception e) {
      stopDrive();
      SmartDashboard.putString("ERROR MESSAGE", e.getMessage());
    }
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  public void resetGyro() {
    m_ahrs.reset();
  }

  public double getGyroAngle() {
    return m_ahrs.getAngle();
  }

  public void resetEncoders() {
    m_drive.resetEncoders();
  }

  public void setRampRate(double rate) {
    m_drive.setRampRate(rate);
  }

  public void setIdleMode(int idleMode) {
    m_drive.setIdleMode(idleMode);
  }
}
