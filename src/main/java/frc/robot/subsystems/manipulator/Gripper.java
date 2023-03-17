// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.tabs;

public class Gripper extends SubsystemBase {
  private static Gripper instance;
  private Pneumatics pneumatics = Pneumatics.getInstance();
  private Boolean isClosing = false;
  private String tab = tabs.kManipulators;

  DoubleSolenoid m_dsol1 =
      pneumatics.add_double_solenoid(
          GripperConstants.kSol1_Channel1, GripperConstants.kSol1_Channel2);
  // the next two are solonoids on the robot from our week 1 competition, we will simply close
  DoubleSolenoid m_dsol2 =
      pneumatics.add_double_solenoid(
          GripperConstants.kSol2_Channel1, GripperConstants.kSol2_Channel2);
  DoubleSolenoid m_dsol3 =
      pneumatics.add_double_solenoid(
          GripperConstants.kSol3_Channel1, GripperConstants.kSol3_Channel2);

  /** Creates a new Gripper. */
  private Gripper() {
    super();
    AddToShuffleboard.add(tab, "Is Gripper Closing", isClosing);
    // make sure the unused solenoids are closed
    // pneumatics.backward(m_dsol2);
    // pneumatics.backward(m_dsol3)
    m_dsol2.close();
    m_dsol3.close();
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
