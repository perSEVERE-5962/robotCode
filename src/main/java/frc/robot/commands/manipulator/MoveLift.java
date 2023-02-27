// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Lift;

public class MoveLift extends CommandBase {
  /** Creates a new Lift. */
  private Lift m_lift;

  private double m_position;

  public MoveLift(double position) {
    m_lift = Lift.getInstance();
    m_position = position;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.moveToPositionWithPID(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double lift = m_lift.getPosition();
    return lift >= m_position - 0.2;
  }
}
