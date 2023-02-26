// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeGripper  extends SubsystemBase{
  private static CubeGripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();

  DoubleSolenoid m_sol = pneumatics.add_double_solenoid(4, 5);

  /** Creates a new Gripper. */
  private CubeGripper() {
    super();
  }

  public void close() {
    SmartDashboard.putString("Driver Action", "closing cube gripper");
    pneumatics.forward(m_sol);
  }

  public void open() {
    SmartDashboard.putString("Driver Action", "opening cube gripper");
    pneumatics.backward(m_sol);
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
