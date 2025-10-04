package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reach;
import frc.robot.subsystems.Pivot;
public class moveSubsystems extends Command {
    private Wrist wristSub;
    private Reach reachSub;
    private Pivot pivotSub;
    private double speed;
    private String sub;
    public moveSubsystems(double speed, String sub){
        wristSub = Wrist.getInstance();
        reachSub = Reach.getInstance();
        pivotSub = Pivot.getInstance();
        this.speed = speed;
        this.sub = sub;
    }
    @Override
  public void initialize() {}

  public void execute() {
    if(sub == "wristSub"){
    wristSub.move(speed);
    }
    if(sub == "pivotSub"){
      if(pivotSub.getPosition()<0.170 && pivotSub.getPosition()>0.130 &&  speed<0){
        pivotSub.move(0);
      }else{
      pivotSub.move(speed);
      }
    }
    if(sub == "reachSub"){
      reachSub.move(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(sub == "wristSub"){
      wristSub.move(0);
    }
    if(sub == "pivotSub"){
      pivotSub.move(0);
    }
    if(sub == "reachSub"){
      reachSub.move(0);
    }
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
