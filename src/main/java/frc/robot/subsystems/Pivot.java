// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Pivot extends Actuator {
  private static Pivot instance;
  private SparkMax followerMotor;
  private SparkMaxConfig followerConfig;

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
        Constants.PivotConstants.kLowerSoftLimit, 
        false,
        true);
        
        followerMotor = new SparkMax(PivotConstants.kFollowerID, SparkLowLevel.MotorType.kBrushless);
        followerConfig = new SparkMaxConfig();
        followerConfig.follow(PivotConstants.kPivotID, true);
        followerConfig.inverted(true);
        followerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        followerConfig.smartCurrentLimit(40);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

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
