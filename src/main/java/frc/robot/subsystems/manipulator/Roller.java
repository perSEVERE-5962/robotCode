// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  private static Roller instance;

  private CANSparkMax m_leadMotor;

  /** Creates a new Roller. */
  public Roller() {
    m_leadMotor =
        new CANSparkMax(
            Constants.CANDeviceIDs.kRollerId,
            com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed);

    m_leadMotor.setInverted(false);

    m_leadMotor
        .getPIDController()
        .setOutputRange(Constants.RollerConstants.kMinOutput, Constants.RollerConstants.kMaxOutput);
  }

  public void moveWithVelocity(double velocity) {
    m_leadMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public static Roller get_instance() {
    if (instance == null) {
      instance = new Roller();
    }

    return instance;
  }
}
