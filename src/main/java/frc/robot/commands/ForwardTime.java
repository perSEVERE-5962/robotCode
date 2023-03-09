// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ForwardTime extends Move {
  private double timeWanted;
  private double m_InitialTime;

  /** Creates a new Forward. */
  public ForwardTime(SwerveSubsystem driveTrain, double translationXSupplier, double timeWanted) {
    super(driveTrain, translationXSupplier, 0, 0);
    this.timeWanted = timeWanted;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_InitialTime = System.currentTimeMillis();
  }

  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_InitialTime) >= timeWanted;
  }
}
