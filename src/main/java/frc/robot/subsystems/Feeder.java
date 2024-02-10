package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Feeder extends SubsystemBase{
     private CANSparkMax intakeMotor;

  /** Creates a new Intake. */
  public Feeder(boolean isinverted,int motorId) {
    intakeMotor = new CANSparkMax(motorId, CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(isinverted);

  }


  public void run(double speed) {
    intakeMotor.set(speed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    
}
