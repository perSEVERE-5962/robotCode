// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.obsolete.commands.manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.obsolete.subsystems.CubeGripper;

public class CubeGripperClose extends CommandBase {
  /** Creates a new CloseManipulator. */
  CubeGripper m_cubegripper;

  public CubeGripperClose() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cubegripper = CubeGripper.getInstance();
    addRequirements(m_cubegripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cubegripper.close();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
