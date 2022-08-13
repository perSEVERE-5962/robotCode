// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.CompetitionRobot;

/** Add your docs here. */
public class RevDrive extends DriveBase {
  private CANSparkMax m_leftLeadMotor;
  private CANSparkMax m_leftFollowerMotor;
  private CANSparkMax m_rightLeadMotor;
  private CANSparkMax m_rightFollowerMotor;

  private RelativeEncoder m_leftLeadEncoder;
  private RelativeEncoder m_leftFollowerEncoder;
  private RelativeEncoder m_rightLeadEncoder;
  private RelativeEncoder m_rightFollowerEncoder;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP); // NavX connected over MXP

  RevDrive() {
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
    m_leftLeadMotor = new CANSparkMax(CompetitionRobot.leftLeadDeviceID, MotorType.kBrushless);
    m_leftFollowerMotor =
        new CANSparkMax(CompetitionRobot.leftFollowerDeviceID, MotorType.kBrushless);
    m_rightLeadMotor = new CANSparkMax(CompetitionRobot.rightLeadDeviceID, MotorType.kBrushless);
    m_rightFollowerMotor =
        new CANSparkMax(CompetitionRobot.rightFollowerDeviceID, MotorType.kBrushless);

    // /**
    // * The RestoreFactoryDefaults method can be used to reset the configuration
    // parameters in the
    // * SPARK MAX to their factory default state. If no argument is passed, these
    // parameters will not
    // * persist between power cycles
    // */
    // m_leftLeadMotor.restoreFactoryDefaults();
    // m_leftFollowerMotor.restoreFactoryDefaults();
    // m_rightLeadMotor.restoreFactoryDefaults();
    // m_rightFollowerMotor.restoreFactoryDefaults();

    /** invert one side of the drive */
    m_leftLeadMotor.setInverted(true);
    m_rightLeadMotor.setInverted(false);

    configurePID(m_leftLeadMotor);
    configurePID(m_rightLeadMotor);

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling the
     * follow() method on the SPARK MAX you want to configure as a follower, and by passing as a
     * parameter the SPARK MAX you want to configure as a leader.
     */
    m_leftFollowerMotor.follow(m_leftLeadMotor);
    m_rightFollowerMotor.follow(m_rightLeadMotor);

    /** create the encoders for each motor each Neo brushless motor has it's own built-in encoder */
    m_leftLeadEncoder = m_leftLeadMotor.getEncoder();
    m_leftFollowerEncoder = m_leftFollowerMotor.getEncoder();
    m_rightLeadEncoder = m_rightLeadMotor.getEncoder();
    m_rightFollowerEncoder = m_rightFollowerMotor.getEncoder();

    init(m_leftLeadMotor, m_rightLeadMotor);
    setRampRate(0);
  }

  private void configurePID(CANSparkMax motor) {
    motor.getPIDController().setP(Constants.DrivePIDCoeffients.kP);
    motor.getPIDController().setI(Constants.DrivePIDCoeffients.kI);
    motor.getPIDController().setD(Constants.DrivePIDCoeffients.kD);
    motor.getPIDController().setIZone(Constants.DrivePIDCoeffients.kIz);
    motor.getPIDController().setFF(Constants.DrivePIDCoeffients.kFF);
    motor
        .getPIDController()
        .setOutputRange(
            Constants.DrivePIDCoeffients.kMinOutput, Constants.DrivePIDCoeffients.kMaxOutput);
  }

  @Override
  public void resetGyroAngle() {
    m_navx.reset();
  }

  @Override
  public double getGyroAngle() {
    return m_navx.getAngle();
  }

  @Override
  public void resetEncoders() {
    m_leftLeadEncoder.setPosition(0);
    m_leftFollowerEncoder.setPosition(0);
    m_rightLeadEncoder.setPosition(0);
    m_rightFollowerEncoder.setPosition(0);
  }

  @Override
  public double getLeftEncoderDistance() {
    return convertPositionToDistance(getAverageLeftEncoderPosition());
  }

  @Override
  public double getRightEncoderDistance() {
    return convertPositionToDistance(getAverageRightEncoderPosition());
  }

  private double getAverageLeftEncoderPosition() {
    return (m_leftLeadEncoder.getPosition() + m_leftFollowerEncoder.getPosition()) / 2;
  }

  private double getAverageRightEncoderPosition() {
    return (m_rightLeadEncoder.getPosition() + m_rightFollowerEncoder.getPosition()) / 2;
  }

  @Override
  public double getAverageEncoderDistance() {
    double encoderPosition =
        (getAverageLeftEncoderPosition() + getAverageRightEncoderPosition()) / 2;

    return convertPositionToDistance(encoderPosition);
  }

  @Override
  public void setRampRate(double rate) {
    m_leftLeadMotor.setOpenLoopRampRate(rate);
    m_leftFollowerMotor.setOpenLoopRampRate(rate);
    m_rightLeadMotor.setOpenLoopRampRate(rate);
    m_rightFollowerMotor.setOpenLoopRampRate(rate);
  }

  public void moveDistanceWithPID(double distance) throws Exception {
    double position = convertDistanceToPosition(distance);
    m_leftLeadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    m_rightLeadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setIdleMode(int idleMode) {
    switch (idleMode) {
      case Constants.MotorControllerIdleModes.kBrake:
        m_leftLeadMotor.setIdleMode(IdleMode.kBrake);
        m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);
        m_rightLeadMotor.setIdleMode(IdleMode.kBrake);
        m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
        break;
      case Constants.MotorControllerIdleModes.kCoast:
        m_leftLeadMotor.setIdleMode(IdleMode.kCoast);
        m_leftFollowerMotor.setIdleMode(IdleMode.kCoast);
        m_rightLeadMotor.setIdleMode(IdleMode.kCoast);
        m_rightFollowerMotor.setIdleMode(IdleMode.kCoast);
        break;
    }
  }
}
