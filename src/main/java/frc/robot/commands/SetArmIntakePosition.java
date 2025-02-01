// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Reach;


public class SetArmIntakePosition extends Command {
  private Reach armSub;
  private double m_targetPos;
  /** Creates a new SetArmIntakePosition. */
  public SetArmIntakePosition() {
    armSub = Reach.getInstance();
    m_targetPos =  Constants.ReachConstants.kLowerSoftLimit;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.moveToPositionWithPID(m_targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armSub.getPosition() <= m_targetPos){
      return true;
    }
    else{
      return false;
    }
  }
}
