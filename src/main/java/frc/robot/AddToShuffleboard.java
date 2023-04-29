// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/*
 * This class exists in order to reduce verbosity whenever a
 * new value needs to be added to the shuffleboard
 */
public class AddToShuffleboard {
  public static GenericEntry add(String tab, String title, Object obj) {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab(tab);
    return shuffleboardTab.add(title, obj).getEntry();
  }
}
