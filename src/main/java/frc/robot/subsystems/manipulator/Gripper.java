// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper  extends SubsystemBase{
  private static Gripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();

  DoubleSolenoid m_dsol1 = pneumatics.add_double_solenoid(0, 1);
  DoubleSolenoid m_dsol2 = pneumatics.add_double_solenoid(2, 3);
  /** Creates a new Gripper. */
  private Gripper() {
    super();
  }

  public void close() {
    SmartDashboard.putString("Co-Pilot Action", "closing gripper");
    pneumatics.forward(m_dsol1);
    pneumatics.forward(m_dsol2);
  }

  public void open() {
    SmartDashboard.putString("Co-Pilot Action", "opening gripper");
    pneumatics.backward(m_dsol1);
    pneumatics.backward(m_dsol2);
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
