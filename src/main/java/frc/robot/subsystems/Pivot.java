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
  private SparkMaxConfig motorConfig;
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
        true,
        false);
        
        followerMotor = new SparkMax(PivotConstants.kFollowerID, SparkLowLevel.MotorType.kBrushless);
        followerConfig = new SparkMaxConfig();
        followerConfig.follow(PivotConstants.kPivotID, true);
        followerConfig.inverted(true);
        followerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        followerConfig.smartCurrentLimit(60);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public void moveToPositionWithPID(double position) {
    if(position>.513){
      motorConfig = getMotorConfig();
      motorConfig.inverted(true);
      followerConfig.inverted(false);
      followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      getArmMotor().configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    else{
      motorConfig = getMotorConfig();
      motorConfig.inverted(false);
      followerConfig.inverted(true);
      followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      getArmMotor().configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    getArmMotor().getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
  }





  public void periodic() {
        double theEncoder=instance.getPosition();
       SmartDashboard.putNumber("Pivot", theEncoder);
  }
//   public void moveToPositionWithPID(double position) {
//     if(Reach.getInstance().getPosition()> 5){
//       getArmMotor().getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
//     }
//     else{
//       getArmMotor().getClosedLoopController().setReference(0.72, SparkMax.ControlType.kPosition);
//     }
// }

  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }
    return instance;
  }
  @Override
  public double getP(){
    return SmartDashboard.getNumber("pivotP", Constants.PivotConstants.kP);
  }
  @Override
  public double getI(){
    return SmartDashboard.getNumber("pivotI", Constants.PivotConstants.kI);
  }
  @Override
  public double getD(){
    return SmartDashboard.getNumber("pivotD", Constants.PivotConstants.kD);
  }
}
