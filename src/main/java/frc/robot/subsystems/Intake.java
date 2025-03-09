package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class Intake extends SubsystemBase{
    private SparkMax intakeMotor;
    private static Intake instance;
    private SparkMaxConfig motorConfig; 
    //private SparkMax followerMotor;
    //private SparkMaxConfig followerConfig;

  /** Creates a new Intake. */
  private Intake() {
    intakeMotor = new SparkMax(Constants.CANDeviceIDs.kIntakeID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig(); 


    
    motorConfig.inverted(true);
    //motorConfig.limitSwitch.forwardLimitSwitchEnabled(true); 
    //motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    motorConfig.smartCurrentLimit(20);
    motorConfig.encoder.velocityConversionFactor(1);
    // motorConfig.closedLoop
    //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //         .p(0.027) //0.5-0.05
    //         .i(0) 
    //         .d(0) 
    //         .outputRange(-1,1) 
    //         .velocityFF(1.0/5767); 
    intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //  followerMotor = new SparkMax(CANDeviceIDs.kFollowerID, SparkLowLevel.MotorType.kBrushless);
    //     followerConfig = new SparkMaxConfig();
    //     followerConfig.follow(Constants.CANDeviceIDs.kIntakeID, true);
    //     followerConfig.inverted(true);
    //     followerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    //     followerConfig.smartCurrentLimit(20);
    //     followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }
  public boolean checkNormallyOpenLimitSwitch(){
    return intakeMotor.getForwardLimitSwitch().isPressed();
  }

  public void run(double speed) {
    intakeMotor.set(speed);
    SmartDashboard.putNumber("intake Current", intakeMotor.getOutputCurrent());
  }

  @Override
  public void periodic() {
 
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  // public void moveToPositionWithPID(double velocity) {
  //   intakeMotor.getClosedLoopController().setReference(velocity, SparkMax.ControlType.kVelocity);
  // }
}
