// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LinearSlide extends SubsystemBase {
  private CANSparkMax m_LinearSlideSpark;

  private RelativeEncoder m_encoder;

  public LinearSlide() {
    m_LinearSlideSpark =
        new CANSparkMax(
            Constants.MotorControllerDeviceID.LinearSlideDeviceID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_LinearSlideSpark.setInverted(false);

    m_encoder = m_LinearSlideSpark.getEncoder();
    m_encoder.setPosition(0);

    m_LinearSlideSpark.getPIDController().setP(Constants.LinearSlidePIDCoefficients.kP);
    m_LinearSlideSpark.getPIDController().setI(Constants.LinearSlidePIDCoefficients.kI);
    m_LinearSlideSpark.getPIDController().setD(Constants.LinearSlidePIDCoefficients.kD);
    m_LinearSlideSpark.getPIDController().setIZone(Constants.LinearSlidePIDCoefficients.kIz);
    m_LinearSlideSpark.getPIDController().setFF(Constants.LinearSlidePIDCoefficients.kFF);

    m_LinearSlideSpark
        .getPIDController()
        .setOutputRange(
            Constants.LinearSlidePIDCoefficients.kMinOutput,
            Constants.LinearSlidePIDCoefficients.kMaxOutput);
  }

  public void moveArm(double speed) {
    m_LinearSlideSpark.set(speed);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveToPositionWithPID(double position) {
    m_LinearSlideSpark.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }
}
