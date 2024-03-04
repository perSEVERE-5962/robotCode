// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Move;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class Turn extends Move {
  private double degreesWanted;
  private double initialYaw;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public Turn(SwerveSubsystem driveTrain, double rotationSpeed, double degreesWanted) {
    super(driveTrain, 0, 0, rotationSpeed);
    this.degreesWanted = degreesWanted;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    initialYaw = driveTrain.getYaw();
    SmartDashboard.putNumber("Initial YaW", initialYaw);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Current Yaw", driveTrain.getYaw());
    if (driveTrain.getYaw() >= Math.abs(initialYaw - degreesWanted)) {
      return true;
    }
    return false;
  }
}
