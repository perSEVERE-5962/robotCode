// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.MotorControllerDeviceID;

/** Add your docs here. */
public class CTREDrive extends DriveBase {
    private WPI_TalonSRX m_leftTalon;
    private WPI_VictorSPX m_leftVictor;
    private WPI_TalonSRX m_rightTalon;
    private WPI_VictorSPX m_rightVictor;

    CTREDrive() {
        m_rightTalon = new WPI_TalonSRX(MotorControllerDeviceID.rightLeadDeviceID);
        m_rightVictor = new WPI_VictorSPX(MotorControllerDeviceID.rightFollowerDeviceID);
        m_leftTalon = new WPI_TalonSRX(MotorControllerDeviceID.leftLeadDeviceID);
        m_leftVictor = new WPI_VictorSPX(MotorControllerDeviceID.leftFollowerDeviceID);
        m_leftVictor.follow(m_leftTalon, FollowerType.PercentOutput);
        m_rightVictor.follow(m_rightTalon, FollowerType.PercentOutput);
        m_leftVictor.setInverted(false);
        m_leftTalon.setInverted(false);
        m_rightVictor.setInverted(true);
        m_rightTalon.setInverted(true);

        m_leftTalon.configOpenloopRamp(0.7);
        m_leftTalon.configClosedloopRamp(0);
        m_rightTalon.configOpenloopRamp(0.7);
        m_rightTalon.configClosedloopRamp(0);

        init(m_leftTalon, m_rightTalon);
    }

    @Override
    public void resetEncoders() {
        m_leftTalon.setSelectedSensorPosition(0);
        m_rightTalon.setSelectedSensorPosition(0);    
    }

    @Override
    public double getLeftEncoderDistance() {
        return m_leftTalon.getSelectedSensorPosition();
    }

    @Override
    public double getRightEncoderDistance() {
        return m_rightTalon.getSelectedSensorPosition();
    }

    @Override
    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
    }

}