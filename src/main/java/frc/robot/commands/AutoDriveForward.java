// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoDriveForward extends CommandBase {
  private double m_distance;
  private Drivetrain m_driveTrain;
  /** Creates a new AutoDriveForward. */
  public AutoDriveForward(double distance, Drivetrain driveTrain) {
    m_distance = distance;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.''
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber(
        "frontLeftModule", m_driveTrain.m_frontLeftModule.getPosition().distanceMeters);
    SmartDashboard.putNumber(
        "frontRightModule", m_driveTrain.m_frontRightModule.getPosition().distanceMeters);
    SmartDashboard.putNumber(
        "backLeftModule", m_driveTrain.m_backLeftModule.getPosition().distanceMeters);
    SmartDashboard.putNumber(
        "backRightModule", m_driveTrain.m_backRightModule.getPosition().distanceMeters);
    // m_driveTrain.tankDrive(-0.5, -0.5);
    // m_driveTrain.moveDistanceWithPID(m_distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    // isFinished = m_driveTrain.Distance() < m_distance;
    return isFinished;
  }
}
