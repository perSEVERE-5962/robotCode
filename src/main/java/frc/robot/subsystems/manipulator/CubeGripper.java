// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;

public class CubeGripper extends SubsystemBase {
  private static CubeGripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();
  private boolean is_closing = false;

  DoubleSolenoid m_sol = pneumatics.add_double_solenoid(4, 5);

  /** Creates a new Gripper. */
  private CubeGripper() {
    super();
    AddToShuffleboard.add("Manipulators", "Is Cube Gripper Closing", is_closing);
  }

  public void close() {
    is_closing = true;
    pneumatics.forward(m_sol);
  }

  public void open() {
    is_closing = false;
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
