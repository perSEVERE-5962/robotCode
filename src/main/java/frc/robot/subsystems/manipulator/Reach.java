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

public class Reach extends SubsystemBase {
  private static Reach instance;

  private CANSparkMax m_leadMotor;
  //  private CANSparkMax m_followMotor;
  private GenericEntry reachPositionEntry;

  private RelativeEncoder m_leadEncoder;

  private Reach() {
    m_leadMotor =
        new CANSparkMax(
            Constants.CANDeviceIDs.kReachID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    // m_followMotor = new CANSparkMax(
    //     Constants.CANDeviceIDs.kReachFollowID,
    //     com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leadMotor.setInverted(false);

    m_leadMotor.getPIDController().setP(Constants.ReachConstants.kP);
    m_leadMotor.getPIDController().setI(Constants.ReachConstants.kI);
    m_leadMotor.getPIDController().setD(Constants.ReachConstants.kD);
    m_leadMotor.getPIDController().setIZone(Constants.ReachConstants.kIz);
    m_leadMotor.getPIDController().setFF(Constants.ReachConstants.kFF);

    m_leadMotor
        .getPIDController()
        .setOutputRange(Constants.ReachConstants.kMinOutput, Constants.ReachConstants.kMaxOutput);

    // m_followMotor.follow(m_leadMotor);

    m_leadEncoder = m_leadMotor.getEncoder();
    m_leadEncoder.setPosition(0);

    // don't let the reach arm travel past the stop points
    m_leadMotor.setSoftLimit(
        SoftLimitDirection.kForward, Constants.ReachConstants.kExtendSoftLimit);
    m_leadMotor.setSoftLimit(
        SoftLimitDirection.kReverse, Constants.ReachConstants.kRetractSoftLimit);

    String tab = Constants.tabs.kManipulators;

    reachPositionEntry = AddToShuffleboard.add(tab, "Reach Position", 0);
  }

  public double getPosition() {
    return m_leadEncoder.getPosition();
  }

  public void moveToPositionWithPID(double position) {
    m_leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    reachPositionEntry.setDouble(m_leadEncoder.getPosition());
  }

  /**
   * @return the instance
   */
  public static Reach getInstance() {
    if (instance == null) {
      instance = new Reach();
    }

    return instance;
  }
}
