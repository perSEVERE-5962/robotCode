// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ForwardDistance extends Move {
  private double distanceWanted;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public ForwardDistance(
      SwerveSubsystem driveTrain, double translationXSupplier, double distanceWanted) {
    super(driveTrain, translationXSupplier, 0, 0);
    this.distanceWanted = distanceWanted;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetDrivePosition();
  }

  @Override
  public boolean isFinished() {
    if (m_driveTrain.getAverageDistanceInches() >= distanceWanted) {
      return true;
    }
    return false;
  }
}
