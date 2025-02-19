package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reach;
public class ResetArmAndWrist extends Command{
    private Wrist wristSub;
    private Reach reachSub;
    public ResetArmAndWrist(){
        wristSub = Wrist.getInstance();
        reachSub = Reach.getInstance();
    }

    @Override
  public void initialize() {}

    @Override
  public void execute() {
    wristSub.moveToPositionWithPID(0);
    reachSub.moveToPositionWithPID(0);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
