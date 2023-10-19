// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheels extends SubsystemBase {
  private CANSparkMax m_leadMotor;
  // private CANSparkMax m_followMotor;
  CANCoder m_encoder;

  /** Creates a new Wheels. */
  public Wheels(int motorId, int encoderId) {
    m_leadMotor =
        new CANSparkMax(motorId, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    // m_followMotor = new CANSparkMax(
    // Constants.CANDeviceIDs.kReachFollowID,
    // com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leadMotor.setInverted(true);

    m_encoder = new CANCoder(encoderId);
  }

  public void turn(double speed) {
    m_leadMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
