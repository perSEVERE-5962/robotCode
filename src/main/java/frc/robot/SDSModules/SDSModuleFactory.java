// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SDSModules;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SDSModuleType;

public class SDSModuleFactory {
  /**
   * Create a drive based on the specified controller type
   *
   * @param sdsModuleType is one of the types defined in {@link
   *     frc.robot.Constants.SDSModuleType}
   * @returns an instance of {@link frc.robot.SDSModules.SDSModuleInterface}
   */
  public SDSModuleInterface createSDSModule(int sdsModuleType) {
    SDSModuleInterface sdsModuleInterface;

    switch (sdsModuleType) {
      case SDSModuleType.kL1:
        sdsModuleInterface = new SDSModuleL1();
        SmartDashboard.putString("Selected SDS Module", "L1");
        break;
      case SDSModuleType.kL3:
        sdsModuleInterface = new SDSModuleL3();
        SmartDashboard.putString("Selected SDS Module", "L3");
        break;
      default:
        sdsModuleInterface = new SDSModuleL2();
        SmartDashboard.putString("Selected SDS Module", "L2");
        break;
    }
    return sdsModuleInterface;
  }
}
