package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.sensors.RockwellUltrasonic;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private static Intake instance;
    private RockwellUltrasonic armUltrasonic;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new CANSparkMax(Constants.CANDeviceIDs.kIntakeID, CANSparkLowLevel.MotorType.kBrushless);
    intakeMotor.setInverted(false);
    armUltrasonic = new RockwellUltrasonic(UltrasonicConstants.kArm_Analog_Channel);

  }
  public RockwellUltrasonic getUltrasonicOne() {
    return armUltrasonic;
  }



  public void run(double speed) {
    intakeMotor.set(speed);

  }
  public boolean isInRange() {
    double range_of_intake = armUltrasonic.getRange();
    // SmartDashboard.putNumber("Feeder Ultrasonic 1", range_of_feeder);
    // SmartDashboard.putNumber("Feeder Ultrasonic 2", range_of_2feeder);
    if (range_of_intake <= Constants.UltrasonicConstants.kMaxRange) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }
}
