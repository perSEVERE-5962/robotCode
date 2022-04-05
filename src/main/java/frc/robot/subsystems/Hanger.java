// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  private CANSparkMax m_angleLeadControl;
  private CANSparkMax m_angleFollowControl;
  private RelativeEncoder m_encoder;

  public Hanger() {
    m_angleLeadControl = new CANSparkMax(
        Constants.MotorControllerDeviceID.angleLeadDeviceID,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_angleFollowControl = new CANSparkMax(
        Constants.MotorControllerDeviceID.angleLeadDeviceID,
        com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_angleLeadControl.setInverted(false);
    m_angleFollowControl.follow(m_angleLeadControl);
    m_encoder = m_angleLeadControl.getEncoder();
    m_encoder.setPosition(0);


    // m_angleControl.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
    // true);
    // m_angleControl.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
    // true);

    // m_angleControl.setSoftLimit(
    // CANSparkMax.SoftLimitDirection.kForward, (float)
    // Constants.HangerPositions.forwardLimit);
    // m_angleControl.setSoftLimit(
    // CANSparkMax.SoftLimitDirection.kReverse, (float)
    // Constants.HangerPositions.reverseLimit);
    m_angleLeadControl.getPIDController().setP(Constants.HangerPIDCoeffients.kP);
    m_angleLeadControl.getPIDController().setI(Constants.HangerPIDCoeffients.kI);
    m_angleLeadControl.getPIDController().setD(Constants.HangerPIDCoeffients.kD);
    m_angleLeadControl.getPIDController().setIZone(Constants.HangerPIDCoeffients.kIz);
    m_angleLeadControl.getPIDController().setFF(Constants.HangerPIDCoeffients.kFF);
    m_angleLeadControl
        .getPIDController()
        .setOutputRange(
            Constants.HangerPIDCoeffients.kMinOutput, Constants.HangerPIDCoeffients.kMaxOutput);

  }

  public void moveHanger(double speed) {
    m_angleLeadControl.set(speed);
  }

  public double getHangerPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveToPositionWithPID(double position) {
    m_angleLeadControl.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }
}