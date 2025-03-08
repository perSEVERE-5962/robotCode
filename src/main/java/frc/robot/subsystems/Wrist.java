// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends Actuator {
  private static Wrist instance;
  private static AbsoluteEncoder absoluteEncoder;
  private Wrist() {
    super(
        Constants.WristConstants.kWristID,
        Constants.WristConstants.kP,
        Constants.WristConstants.kI,
        Constants.WristConstants.kD,
        Constants.WristConstants.kMinOutput,
        Constants.WristConstants.kMaxOutput,
        Constants.WristConstants.kFF,
        Constants.WristConstants.kIz,
        Constants.WristConstants.kUpperSoftLimit,
        Constants.WristConstants.kLowerSoftLimit,
        true,
        false);

        
  }
  @Override
  public void periodic() {
      double theEncoder=instance.getPosition();
       SmartDashboard.putNumber("Wrist", theEncoder);
  }

  public static Wrist getInstance() {
    if (instance == null) {
      instance = new Wrist();
    }
    return instance;
  }
  public double getP(){
    return SmartDashboard.getNumber("wristP", Constants.WristConstants.kP);
  }
  @Override
  public double getI(){
    return SmartDashboard.getNumber("wristI", Constants.WristConstants.kI);
  }
  @Override
  public double getD(){
    return SmartDashboard.getNumber("wristD", Constants.WristConstants.kD);
  }
}
