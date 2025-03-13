package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reach;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
public class ResetArmAndWrist extends Command{
    private Wrist wristSub;
    private Reach reachSub;
    private Pivot pivotSub;
    private Intake intakeSub;
    private double postion;
    public ResetArmAndWrist(double postion){
        wristSub = Wrist.getInstance();
        reachSub = Reach.getInstance();
        pivotSub = Pivot.getInstance();
        intakeSub = Intake.getInstance();
        addRequirements(intakeSub,wristSub,reachSub, pivotSub);
        this.postion=postion;
       }

    @Override
  public void initialize() {}

    @Override
  public void execute() {
    reachSub.moveToPositionWithPID(-10.0);
    intakeSub.run(0);
    wristSub.moveToPositionWithPID(postion);
    
    pivotSub.moveToPositionWithPID(Constants.PivotConstants.koffSet);
    reachSub.moveToPositionWithPID(-2.0);
    
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
