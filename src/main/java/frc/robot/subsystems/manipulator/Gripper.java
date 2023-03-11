// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private static Gripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();
  private Boolean isClosing = false;
  private String tab = Constants.tabs.kManipulators;

  DoubleSolenoid m_dsol1 = pneumatics.add_double_solenoid(4, 5);
  // DoubleSolenoid m_dsol2 = pneumatics.add_double_solenoid(2, 3);
  /** Creates a new Gripper. */
  private Gripper() {
    super();
    AddToShuffleboard.add(tab, "Is Gripper Closing", isClosing);
  }

  public void close() {
    isClosing = true;
    pneumatics.forward(m_dsol1);
    // pneumatics.forward(m_dsol2);
  }

  public void open() {
    isClosing = false;
    pneumatics.backward(m_dsol1);
    // pneumatics.backward(m_dsol2);
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
