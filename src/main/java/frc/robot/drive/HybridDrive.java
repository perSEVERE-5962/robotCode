// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.MotorControllerDeviceID;

/**
 * Combination of Neo and CIM motors controlled by the SparkMax The Neo is the "lead" The CIM is the
 * "follower" and has no encoder
 */
public class HybridDrive extends DriveBase {
  private CANSparkMax m_leftLeadMotor; // Neo
  private CANSparkMax m_leftFollowerMotor;  // CIM
  private CANSparkMax m_rightLeadMotor; //Neo
  private CANSparkMax m_rightFollowerMotor; // CIM

  private RelativeEncoder m_leftLeadEncoder;
  private RelativeEncoder m_rightLeadEncoder;

  HybridDrive() {
    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
     *
     * <p>The CAN ID, which can be configured using the SPARK MAX Client, is passed as the first
     * parameter
     *
     * <p>The motor type is passed as the second parameter. Motor type can either be:
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
     * com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
     *
     * <p>The example below initializes brushless motors with MotorControllerDeviceID. Change these
     * parameters to match your setup
     */
    m_leftLeadMotor =
        new CANSparkMax(MotorControllerDeviceID.leftLeadDeviceID, MotorType.kBrushless);
    m_leftFollowerMotor =
        new CANSparkMax(MotorControllerDeviceID.leftFollowerDeviceID, MotorType.kBrushed);
    m_rightLeadMotor =
        new CANSparkMax(MotorControllerDeviceID.rightLeadDeviceID, MotorType.kBrushless);
    m_rightFollowerMotor =
        new CANSparkMax(MotorControllerDeviceID.rightFollowerDeviceID, MotorType.kBrushed);

    // /**
    //  * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
    //  * SPARK MAX to their factory default state. If no argument is passed, these parameters will not
    //  * persist between power cycles
    //  */
    // m_leftLeadMotor.restoreFactoryDefaults();
    // m_leftFollowerMotor.restoreFactoryDefaults();
    // m_rightLeadMotor.restoreFactoryDefaults();
    // m_rightFollowerMotor.restoreFactoryDefaults();

    /** invert one side of the drive */
    m_leftLeadMotor.setInverted(true);
    m_rightLeadMotor.setInverted(false);

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling the
     * follow() method on the SPARK MAX you want to configure as a follower, and by passing as a
     * parameter the SPARK MAX you want to configure as a leader.
     */
    m_leftFollowerMotor.follow(m_leftLeadMotor);
    m_rightFollowerMotor.follow(m_rightLeadMotor);

    /** create the encoders for each motor each Neo brushless motor has it's own built-in encoder */
    m_leftLeadEncoder = m_leftLeadMotor.getEncoder();
    m_rightLeadEncoder = m_rightLeadMotor.getEncoder();

    init(m_leftLeadMotor, m_rightLeadMotor);
    setRampRate(0);
  }

  @Override
  public void resetEncoders() {
    m_leftLeadEncoder.setPosition(0);
    m_rightLeadEncoder.setPosition(0);
  }

  @Override
  public double getLeftEncoderDistance() {
    return convertPostitionToDistance(getAverageLeftEncoderPosition());
  }

  @Override
  public double getRightEncoderDistance() {
    return convertPostitionToDistance(getAverageRightEncoderPosition());
  }

  private double getAverageLeftEncoderPosition() {
    return m_leftLeadEncoder.getPosition();
  }

  private double getAverageRightEncoderPosition() {
    return m_rightLeadEncoder.getPosition();
  }

  @Override
  public double getAverageEncoderDistance() {
    double encoderPosition =
        (getAverageLeftEncoderPosition() + getAverageRightEncoderPosition()) / 2;

    return convertPostitionToDistance(encoderPosition);
  }

  @Override
  public void setRampRate(double rate) {
    m_leftLeadMotor.setOpenLoopRampRate(rate);
    m_leftFollowerMotor.setOpenLoopRampRate(rate);
    m_rightLeadMotor.setOpenLoopRampRate(rate);
    m_rightFollowerMotor.setOpenLoopRampRate(rate);
  }
}
