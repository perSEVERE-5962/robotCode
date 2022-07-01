/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.DriveFactory;
import frc.robot.drive.DriveInterface;
import frc.robot.gyro.GyroFactory;
import frc.robot.gyro.GyroInterface;
import java.util.function.DoubleSupplier;

public class DriveTrain extends SubsystemBase {
  private DriveInterface m_drive;
  private boolean driveTrainSet = false;

  public DriveTrain() {
  }

  public void setMotorControllerType(int motorControllerType) {
    if (driveTrainSet == false) {
      DriveFactory driveFactory = new DriveFactory();
      m_drive = driveFactory.createDrive(motorControllerType);
      m_drive.resetEncoders();
      GyroFactory gyroFactory = new GyroFactory();
      GyroInterface gyro = gyroFactory.createGyro(motorControllerType);
      m_drive.setGyro(gyro);
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

  public void swerveDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    m_drive.swerveDrive(translationXSupplier, translationYSupplier, rotationSupplier);
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

  // public void resetGyro() {
  // m_gyro.resetGyro();
  // }

  // public double getGyroAngle() {
  // return m_gyro.getGyroAngle();
  // }

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
