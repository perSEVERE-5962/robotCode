// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CubeGripper extends Pneumatics {
  private static CubeGripper instance;

  DoubleSolenoid m_sol = add_double_solenoid(4, 5);

  /** Creates a new Gripper. */
  private CubeGripper() {
    super();
  }

  public void close() {
    SmartDashboard.putString("Driver Action", "closing cube gripper");
    forward(m_sol);
  }

  public void open() {
    SmartDashboard.putString("Driver Action", "opening cube gripper");
    backward(m_sol);
  }

  /**
   * @return the instance
   */
  public static CubeGripper getInstance() {
    if (instance == null) {
      instance = new CubeGripper();
    }

    return instance;
  }
}
