// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  private static Roller instance;

  private CANSparkMax m_leadMotor;

  public int invertRoller = 1; // 1 is not inverted, -1 is inverted
  private GenericEntry rollerEntry;

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

    rollerEntry = AddToShuffleboard.add("Roller", "LT Intake Cone", true);
  }

  public void moveWithVoltage(double voltage) {
    m_leadMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
  }

  public void periodic() {
    // Not quite what it appears. What this actually does is show whenever left trigger is going to intake the cone.
    rollerEntry.setBoolean(invertRoller == Constants.RollerConstants.kIntakeCone ? true : false); // Even I don't know why I'm doing it like this
  }

  public static Roller get_instance() {
    if (instance == null) {
      instance = new Roller();
    }

    return instance;
  }
}
