// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.Roller;

public class MoveRoller extends CommandBase {
  /** Creates a new MoveRoller. */
  private Roller m_Roller;
  private double velocity;
  public MoveRoller(double velocity) {
    m_Roller = Roller.get_instance();
    this.velocity = velocity;
    addRequirements(m_Roller);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Roller.moveWithVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
