// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private CANSparkMax m_ArmSpark;
  private RelativeEncoder m_encoder;

  public Arm() {
    m_ArmSpark =
        new CANSparkMax(
            Constants.MotorControllerDeviceID.armDeviceID,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    m_encoder = m_ArmSpark.getEncoder();
    m_encoder.setPosition(0);
  }

  public void moveArm(double speed) {
    m_ArmSpark.set(speed);
    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    // -20 is lowest
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
