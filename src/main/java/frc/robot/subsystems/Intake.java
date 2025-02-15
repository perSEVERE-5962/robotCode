package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private SparkMax intakeMotor;
    private static Intake instance;
    private SparkMaxConfig motorConfig; 

  /** Creates a new Intake. */
  private Intake() {
    intakeMotor = new SparkMax(Constants.CANDeviceIDs.kIntakeID, SparkLowLevel.MotorType.kBrushless);
    motorConfig = new SparkMaxConfig(); 

    
    motorConfig.inverted(false);
    motorConfig.limitSwitch.forwardLimitSwitchEnabled(true); 
    motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    intakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }
  public boolean checkNormallyOpenLimitSwitch(){
    return intakeMotor.getForwardLimitSwitch().isPressed();
  }

  public void run(double speed) {
    intakeMotor.set(speed);

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
}
