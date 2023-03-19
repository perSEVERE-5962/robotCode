// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ForwardWithGyro extends Move {
  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public ForwardWithGyro(SwerveSubsystem driveTrain, double translationXSupplier) {
    super(driveTrain, translationXSupplier, 0, 0);

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    if (m_driveTrain.getPitch() <= Constants.PITCH_LEVEL) {
      return true;
    }
    return false;
  }
}
