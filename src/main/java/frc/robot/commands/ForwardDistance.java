// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drivetrain.Drivetrain;

public class ForwardDistance extends Forward {

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  private double m_distance;
  /** Creates a new Forward. */
  public ForwardDistance(Drivetrain driveTrain, double speed, double distance) {
    super(driveTrain, speed);
    m_distance = distance;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getAverageEncoder() >= m_distance;
  }
}
