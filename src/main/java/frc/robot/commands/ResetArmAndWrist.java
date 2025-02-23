package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reach;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants;
public class ResetArmAndWrist extends Command{
    private Wrist wristSub;
    private Reach reachSub;
    private Pivot pivotSub;
    public ResetArmAndWrist(){
        wristSub = Wrist.getInstance();
        reachSub = Reach.getInstance();
        pivotSub = Pivot.getInstance();
    }

    @Override
  public void initialize() {}

    @Override
  public void execute() {
    wristSub.moveToPositionWithPID(Constants.WristConstants.koffSet);
    pivotSub.moveToPositionWithPID(Constants.PivotConstants.koffSet);
    reachSub.move(-0.25);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
