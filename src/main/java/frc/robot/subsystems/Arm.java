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

public class Arm extends SubsystemBase{
    private SparkMax armMotor;
    private SparkMaxConfig motorConfig; 
    private static Arm instance;
    private static RelativeEncoder armEncoder;

    public Arm(){
        armMotor = new SparkMax(Constants.CANDeviceIDs.kArmID, SparkLowLevel.MotorType.kBrushless);
        motorConfig = new SparkMaxConfig(); 
    
        motorConfig.inverted(false); 
        armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        motorConfig.closedLoop 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
        .p(Constants.ArmConstants.kP) 
        .i(Constants.ArmConstants.kI) 
        .d(Constants.ArmConstants.kD) 
        .outputRange(Constants.ArmConstants.kMinOutput, Constants.ArmConstants.kMaxOutput) 
        .velocityFF(Constants.ArmConstants.kFF) 
        .iZone(Constants.ArmConstants.kIz); 

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.forwardSoftLimit(Constants.ArmConstants.kUpperSoftLimit);
        softLimitConfig.reverseSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimit(Constants.ArmConstants.kUpperSoftLimit);

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
    public static Arm getInstance() {
        if (instance == null) {
          instance = new Arm();
        }
    
        return instance;
      }
}
