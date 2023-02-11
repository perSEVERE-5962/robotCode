// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ForwardDistance extends Move {
  private double distanceWanted;
  private double m_InitialDistance;
  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public ForwardDistance(
      Drivetrain driveTrain, double translationXSupplier, double distanceWantedMeters) {
    super(driveTrain, translationXSupplier, 0, 0);
    this.distanceWanted = distanceWantedMeters;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_InitialDistance = m_driveTrain.getAverageEncoder();
    SmartDashboard.putNumber("Initial Encoder Avg", m_driveTrain.getAverageEncoder());
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Current Encoder Avg", m_driveTrain.getAverageEncoder());
    if (distanceWanted <= (m_driveTrain.getAverageEncoder() - m_InitialDistance)) return true;

    return false;
  }
}
