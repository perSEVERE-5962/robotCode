// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static WPI_VictorSPX m_motorControl;

  /** Creates a new Intake. */
  public Intake() {
    m_motorControl = new WPI_VictorSPX(Constants.MotorControllerDeviceID.intakeDeviceID);
    m_motorControl.setInverted(true);
  }

  public void armIntake(double speed) {
    m_motorControl.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
