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

public class Lift extends SubsystemBase {
  private static Lift instance;

  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;

  private RelativeEncoder m_leadEncoder;
  // private RelativeEncoder m_followEncoder;
  private GenericEntry liftPositionEntry;

  private Lift() {
    // the lift uses two Neo motors connected to a single gearbox
    // we will configure one as the leader and the other the follower
    m_leadMotor =
        new CANSparkMax(
            Constants.CANDeviceIDs.kLiftLeadID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_followMotor =
        new CANSparkMax(
            Constants.CANDeviceIDs.kLiftFollowID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leadMotor.setInverted(false);

    m_leadMotor.getPIDController().setP(Constants.LiftConstants.kP);
    m_leadMotor.getPIDController().setI(Constants.LiftConstants.kI);
    m_leadMotor.getPIDController().setD(Constants.LiftConstants.kD);
    m_leadMotor.getPIDController().setIZone(Constants.LiftConstants.kIz);
    m_leadMotor.getPIDController().setFF(Constants.LiftConstants.kFF);

    m_leadMotor
        .getPIDController()
        .setOutputRange(Constants.LiftConstants.kMinOutput, Constants.LiftConstants.kMaxOutput);

    m_followMotor.follow(m_leadMotor);

    m_leadEncoder = m_leadMotor.getEncoder();
    m_leadEncoder.setPosition(0);

    // don't let the lift arm travel past the stop points
    m_leadMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.LiftConstants.kRaiseSoftLimit);
    m_leadMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.LiftConstants.kLowerSoftLimit);

    String tab = Constants.tabs.kManipulators;

    liftPositionEntry = AddToShuffleboard.add(tab, "Lift Position", getPosition());
  }

  public double getPosition() {
    return m_leadEncoder.getPosition();
  }

  public void moveToPositionWithPID(double position) {
    m_leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    liftPositionEntry.setDouble(m_leadEncoder.getPosition());
  }

  /**
   * @return the instance
   */
  public static Lift getInstance() {
    if (instance == null) {
      instance = new Lift();
    }

    return instance;
  }
}
