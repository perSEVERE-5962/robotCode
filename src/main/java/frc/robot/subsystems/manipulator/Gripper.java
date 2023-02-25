// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gripper extends Pneumatics {
  private static Gripper instance;

  DoubleSolenoid m_dsol1 = add_double_solenoid(0, 1);
  DoubleSolenoid m_dsol2 = add_double_solenoid(2, 3);
  /** Creates a new Gripper. */
  private Gripper() {
    super();
  }

  public void close() {
    SmartDashboard.putString("Co-Pilot Action", "closing gripper");
    forward(m_dsol1);
    forward(m_dsol2);
  }

  public void open() {
    SmartDashboard.putString("Co-Pilot Action", "opening gripper");
    backward(m_dsol1);
    backward(m_dsol2);
  }

  /**
   * @return the instance
   */
  public static Gripper getInstance() {
    if (instance == null) {
      instance = new Gripper();
    }

    return instance;
  }
}
