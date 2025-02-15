// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {
    private SparkMax armMotor;
    private SparkMaxConfig motorConfig; 
    private RelativeEncoder armEncoder;
    
    public Actuator(int ID, double P, double I, double D, double MinOutput, double MaxOutput, double FF, double Iz, float kUpperSoftLimit,float kLowerSoftLimit, boolean useThroughBoreEncoder){

        armMotor = new SparkMax(ID, SparkLowLevel.MotorType.kBrushless);
        motorConfig = new SparkMaxConfig(); 
    
        motorConfig.inverted(false); 
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        FeedbackSensor feedBackSensor = FeedbackSensor.kPrimaryEncoder;
         
        if(useThroughBoreEncoder == true){
            feedBackSensor = FeedbackSensor.kAlternateOrExternalEncoder;
        }
        motorConfig.closedLoop
            .feedbackSensor(feedBackSensor)
            .p(P) 
            .i(I) 
            .d(D) 
            .outputRange(MinOutput,MaxOutput) 
            .velocityFF(FF) 
            .iZone(Iz); 
        if(useThroughBoreEncoder == true){
            armEncoder = armMotor.getAlternateEncoder();
        }else{
            armEncoder = armMotor.getEncoder();
        }
        armEncoder.setPosition(0);
        
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.forwardSoftLimit(kUpperSoftLimit);
        softLimitConfig.reverseSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimit(kLowerSoftLimit);


        motorConfig.apply(softLimitConfig);
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
   
    public void periodic() {
        //nothing here
    }

    public double getPosition() {
           return armEncoder.getPosition(); 
        
    }
    public void moveToPositionWithPID(double position) {
      armMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }
}
