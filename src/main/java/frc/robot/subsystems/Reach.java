package frc.robot.subsystems;
import frc.robot.Constants;

public class Reach extends Actuator{

    private static Reach instance;

    public Reach(){
        super(Constants.ReachConstants.kReachID, Constants.ReachConstants.kP, Constants.ReachConstants.kI, Constants.ReachConstants.kD, Constants.ReachConstants.kMinOutput, Constants.ReachConstants.kMaxOutput, Constants.ReachConstants.kFF, Constants.ReachConstants.kIz, Constants.ReachConstants.kUpperSoftLimit);
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
