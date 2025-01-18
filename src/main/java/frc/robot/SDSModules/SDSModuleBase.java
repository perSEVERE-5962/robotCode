// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SDSModules;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public abstract class SDSModuleBase implements SDSModuleInterface {

  protected final double kWheelDiameterMeters = Units.inchesToMeters(3.9);
  protected final double kWheelDiameterInches = 3.9;

  @Override
  public double getWheelDiameterInches() {
    return kWheelDiameterInches;
  }

}
