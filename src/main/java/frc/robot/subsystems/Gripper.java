// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Gripper extends Pneumatics {
  DoubleSolenoid m_dsol1 = add_double_solenoid(0, 1);
  DoubleSolenoid m_dsol2 = add_double_solenoid(2, 3);
  /** Creates a new Gripper. */
  public Gripper() {
    super();
  }

  public void close() {
    forward(m_dsol1);
    forward(m_dsol2);
  }

  public void open() {
    backward(m_dsol1);
    backward(m_dsol2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
