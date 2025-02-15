// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Pivot extends Actuator {
  private static Pivot instance;

  private Pivot() {
    super(
        Constants.PivotConstants.kPivotID,
        Constants.PivotConstants.kP,
        Constants.PivotConstants.kI,
        Constants.PivotConstants.kD,
        Constants.PivotConstants.kMinOutput,
        Constants.PivotConstants.kMaxOutput,
        Constants.PivotConstants.kFF,
        Constants.PivotConstants.kIz,
        Constants.PivotConstants.kUpperSoftLimit,
        Constants.PivotConstants.kLowerSoftLimit);
        
  }
@Override
  public void periodic() {
        double theEncoder=instance.getPosition();
       SmartDashboard.putNumber("Pivot", theEncoder);
  }

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }
}
