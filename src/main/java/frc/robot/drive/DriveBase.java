// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Add your docs here. */
public abstract class DriveBase implements DriveInterface {
        
    // Set up the differential drive controller
    private DifferentialDrive m_diffDrive;
    private MotorController m_leftMotor, m_rightMotor;

    public void init (MotorController leftMotor, MotorController rightMotor){
        m_diffDrive = new DifferentialDrive(leftMotor, rightMotor);
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
    }

    @Override
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_diffDrive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
        m_diffDrive.feed();
    }

    @Override
    public void arcadeDrive(double xSpeed, double zRotation) {
        m_diffDrive.arcadeDrive(xSpeed,zRotation);    
    }

}