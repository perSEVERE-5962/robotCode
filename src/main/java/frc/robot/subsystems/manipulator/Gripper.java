// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;

public class Gripper extends SubsystemBase {
  private static Gripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();
  private Boolean is_closing = false;

  DoubleSolenoid m_dsol1 = pneumatics.add_double_solenoid(0, 1);
  DoubleSolenoid m_dsol2 = pneumatics.add_double_solenoid(2, 3);
  /** Creates a new Gripper. */
  private Gripper() {
    super();
    AddToShuffleboard.add("Manipulators", "Is Gripper Closing", is_closing);
  }

  public void close() {
    is_closing = true;
    pneumatics.forward(m_dsol1);
    pneumatics.forward(m_dsol2);
  }

  public void open() {
    is_closing = false;
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
