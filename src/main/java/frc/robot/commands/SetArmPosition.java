// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Reach;
import frc.robot.subsystems.Wrist;

public class SetArmPosition extends Command {
  private Reach reachSub;
  private int targetPos;
  private Pivot pivotSub;
  private Wrist wristSub;
  /** Creates a new SetArmShootPosition. */
  public SetArmPosition(int scorePostion) {
    reachSub = Reach.getInstance();
    wristSub = Wrist.getInstance();
    pivotSub = Pivot.getInstance();

    targetPos =  scorePostion;
    addRequirements(reachSub,wristSub,pivotSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    reachSub.moveToPositionWithPID(ScoringConstants.postions[targetPos][ScoringConstants.kReach]);
    wristSub.moveToPositionWithPID(ScoringConstants.postions[targetPos][ScoringConstants.kWrist]);
    pivotSub.moveToPositionWithPID(ScoringConstants.postions[targetPos][ScoringConstants.kPivot]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(reachSub.getPosition() >= m_targetPos){
    //   return true;
    // }
    // else{
    //   return false;
    // }
    return true;
  }
}
