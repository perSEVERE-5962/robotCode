// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ForwardWithGyro extends Move {
  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  private double pitchWanted;
  private boolean useGreaterThan = false;
  /** Creates a new Forward. */
  public ForwardWithGyro(
      SwerveSubsystem driveTrain,
      double translationXSupplier,
      double pitchWanted,
      boolean useGreaterThan) {
    super(driveTrain, translationXSupplier, 0, 0);
    this.pitchWanted = pitchWanted;
    if (useGreaterThan) {
      this.useGreaterThan = true;
    }
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    if (useGreaterThan && m_driveTrain.getPitch() >= this.pitchWanted) {
      return true;
    } else if (!useGreaterThan && m_driveTrain.getPitch() <= this.pitchWanted) {
      return true;
    }
    return false;
  }
}
