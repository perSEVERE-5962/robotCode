/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.DriveFactory;
import frc.robot.drive.DriveInterface;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {

  private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
  private DriveInterface m_drive;

  public AHRS getGyro(){
    return m_ahrs;
  }

  public DriveTrain() {
    DriveFactory driveFactory = new DriveFactory();
    m_drive = driveFactory.createDrive();
  }

  public double getLeftEncoderDistance(){
    return m_drive.getLeftEncoderDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void tankDrive(double leftAxis, double rightAxis) {
    m_drive.tankDrive(leftAxis, rightAxis);
  }

  public void arcadeDrive(double leftAxis, double rightAxis) {
    m_drive.tankDrive(leftAxis, rightAxis);
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

  public void resetEncoders(){
    m_drive.resetEncoders();
  }

}