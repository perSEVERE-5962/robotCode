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
    public moveSubsystems(double speed){
        wristSub = Wrist.getInstance();
        reachSub = Reach.getInstance();
        pivotSub = Pivot.getInstance();
        this.speed = speed;
    }
    @Override
  public void initialize() {}

  public void execute() {
    //wristSub.move(0);
    //pivotSub.move(0);
    reachSub.move(speed);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
