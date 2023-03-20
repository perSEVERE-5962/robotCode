// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private static Wrist instance;

  private CANSparkMax m_leadMotor;
  //  private CANSparkMax m_followMotor;
  private GenericEntry wristPositionEntry;

  private RelativeEncoder m_leadEncoder;

  private Wrist() {
    m_leadMotor =
        new CANSparkMax(
            Constants.CANDeviceIDs.kWristID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    // m_followMotor = new CANSparkMax(
    //     Constants.CANDeviceIDs.kReachFollowID,
    //     com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leadMotor.setInverted(false);

    m_leadMotor.getPIDController().setP(Constants.WristConstants.kP);
    m_leadMotor.getPIDController().setI(Constants.WristConstants.kI);
    m_leadMotor.getPIDController().setD(Constants.WristConstants.kD);
    m_leadMotor.getPIDController().setIZone(Constants.WristConstants.kIz);
    m_leadMotor.getPIDController().setFF(Constants.WristConstants.kFF);

    m_leadMotor
        .getPIDController()
        .setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);

    // m_followMotor.follow(m_leadMotor);

    m_leadEncoder = m_leadMotor.getEncoder();
    m_leadEncoder.setPosition(0);

    // don't let the wrist travel past the stop points
    m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.WristConstants.kLowerSoftLimit);
    m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.WristConstants.kRaiseSoftLimit);

    String tab = Constants.tabs.kManipulators;

    wristPositionEntry = AddToShuffleboard.add(tab, "Wrist Position", 0);
  }

  public double getPosition() {
    return m_leadEncoder.getPosition();
  }

  public void move(double speed) {
    m_leadMotor.set(speed);
  }

  public void moveToPositionWithPID(double position) {
    m_leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    wristPositionEntry.setDouble(m_leadEncoder.getPosition());
  }

  /**
   * @return the instance
   */
  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }

    return instance;
  }
}
