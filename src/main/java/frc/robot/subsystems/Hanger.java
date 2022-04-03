// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  private CANSparkMax m_angleControl;
  private static WPI_TalonSRX m_telescopeControl;
  private RelativeEncoder m_encoder;

  public Hanger() {
    m_angleControl =
        new CANSparkMax(
            Constants.MotorControllerDeviceID.angleDeviceID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_angleControl.setInverted(false);
    m_encoder = m_angleControl.getEncoder();
    m_encoder.setPosition(0);

    // m_angleControl.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // m_angleControl.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // m_angleControl.setSoftLimit(
    //     CANSparkMax.SoftLimitDirection.kForward, (float) Constants.HangerPositions.forwardLimit);
    // m_angleControl.setSoftLimit(
    //     CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.HangerPositions.reverseLimit);
    m_angleControl.getPIDController().setP(Constants.HangerPIDCoeffients.kP);
    m_angleControl.getPIDController().setI(Constants.HangerPIDCoeffients.kI);
    m_angleControl.getPIDController().setD(Constants.HangerPIDCoeffients.kD);
    m_angleControl.getPIDController().setIZone(Constants.HangerPIDCoeffients.kIz);
    m_angleControl.getPIDController().setFF(Constants.HangerPIDCoeffients.kFF);
    m_angleControl
        .getPIDController()
        .setOutputRange(
            Constants.HangerPIDCoeffients.kMinOutput, Constants.HangerPIDCoeffients.kMaxOutput);

    m_telescopeControl = new WPI_TalonSRX(Constants.MotorControllerDeviceID.telescopingDeviceID);
    m_telescopeControl.setInverted(true);
  }

  public void moveHanger(double speed) {
    m_angleControl.set(speed);
  }

  public void telescopeControl(double speed) {
    m_telescopeControl.set(speed);
  }

  public double getHangerPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveToPositionWithPID(double position) {
    m_angleControl.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }
}