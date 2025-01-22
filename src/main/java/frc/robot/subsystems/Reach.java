package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;

public class Reach extends SubsystemBase{
    private SparkMax armMotor;
    private SparkMaxConfig motorConfig; 
    private static Reach instance;
    private static RelativeEncoder armEncoder;

    public Reach(){

        armMotor = new SparkMax(Constants.ReachConstants.kReachID, SparkLowLevel.MotorType.kBrushless);
        motorConfig = new SparkMaxConfig(); 
    
        motorConfig.inverted(false); 
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        motorConfig.closedLoop 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
        .p(Constants.ReachConstants.kP) 
        .i(Constants.ReachConstants.kI) 
        .d(Constants.ReachConstants.kD) 
        .outputRange(Constants.ReachConstants.kMinOutput, Constants.ReachConstants.kMaxOutput) 
        .velocityFF(Constants.ReachConstants.kFF) 
        .iZone(Constants.ReachConstants.kIz); 

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.forwardSoftLimit(Constants.ReachConstants.kUpperSoftLimit);
        softLimitConfig.reverseSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimit(Constants.ReachConstants.kUpperSoftLimit);

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
    public static Reach getInstance() {
        if (instance == null) {
          instance = new Reach();
        }
    
        return instance;
      }
}
