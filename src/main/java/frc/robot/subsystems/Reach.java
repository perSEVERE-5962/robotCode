package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.AbsoluteEncoder;


public class Reach extends Actuator {

  private static Reach instance;

  private Reach() {
    super(
        Constants.ReachConstants.kReachID,
        Constants.ReachConstants.kP,
        Constants.ReachConstants.kI,
        Constants.ReachConstants.kD,
        Constants.ReachConstants.kMinOutput,
        Constants.ReachConstants.kMaxOutput,
        Constants.ReachConstants.kFF,
        Constants.ReachConstants.kIz,
        Constants.ReachConstants.kUpperSoftLimit,
        Constants.ReachConstants.kLowerSoftLimit,
        false);
  }
  @Override
  public void periodic() {
        double theEncoder=instance.getPosition();
       SmartDashboard.putNumber("Reach", theEncoder);
  }

  public static Reach getInstance() {
    if (instance == null) {
      instance = new Reach();
    }

    return instance;
  }
}
