// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Roller;

public class MoveRollerAuto extends CommandBase {
  private Roller m_roller;
  private boolean intakeCone = false;
  private int duration = 0;
  private long initialTime = 0;
  /** Creates a new MoveRollerAuto. */
  public MoveRollerAuto(boolean intakeCone, int duration) {
    m_roller = Roller.get_instance();
    this.intakeCone = intakeCone;
    this.duration = duration;
    //addRequirements(m_roller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int multiplier = intakeCone ? -1 : 1;
    m_roller.moveWithVoltage(Constants.RollerConstants.kMaxVoltage * multiplier * m_roller.invertRoller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_roller.moveWithVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((System.currentTimeMillis() - initialTime) >= duration) {
      m_roller.moveWithVoltage(0);
      return true;
    }
    return false;
  }
}
