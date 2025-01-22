// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    private SparkMax armMotor;
    private SparkMaxConfig motorConfig; 
    private static Pivot instance;
    private static RelativeEncoder armEncoder;


    public Pivot(){

        armMotor = new SparkMax(Constants.PivotConstants.kPivotID, SparkLowLevel.MotorType.kBrushless);
        motorConfig = new SparkMaxConfig(); 
    
        motorConfig.inverted(false); 
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        motorConfig.closedLoop 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
        .p(Constants.PivotConstants.kP) 
        .i(Constants.PivotConstants.kI) 
        .d(Constants.PivotConstants.kD) 
        .outputRange(Constants.PivotConstants.kMinOutput, Constants.PivotConstants.kMaxOutput) 
        .velocityFF(Constants.PivotConstants.kFF) 
        .iZone(Constants.PivotConstants.kIz); 

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.forwardSoftLimit(Constants.PivotConstants.kUpperSoftLimit);
        softLimitConfig.reverseSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimit(Constants.PivotConstants.kUpperSoftLimit);

        motorConfig.apply(softLimitConfig);
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getPosition() {
        return armEncoder.getPosition();
    }

    public void moveToPositionWithPID(double position) {
      armMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }
    
    @Override
    public void periodic(){

    }
    public static Pivot getInstance() {
        if (instance == null) {
          instance = new Pivot();
        }
    
        return instance;
      }
}
