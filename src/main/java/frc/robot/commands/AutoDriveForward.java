// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AutoDriveForward extends CommandBase {
  // private double m_distance;
  private SwerveSubsystem m_driveTrain;
  /** Creates a new AutoDriveForward. */
  public AutoDriveForward(double Distance, SwerveSubsystem driveTrain) {
    // m_distance = Distance;
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
    SwerveModulePosition[] swerveModulePosition = m_driveTrain.getSwerveModulePositions();
    SmartDashboard.putNumber("frontLeft", swerveModulePosition[0].distanceMeters);
    SmartDashboard.putNumber("frontRight", swerveModulePosition[1].distanceMeters);
    SmartDashboard.putNumber("backLeft", swerveModulePosition[2].distanceMeters);
    SmartDashboard.putNumber("backRight", swerveModulePosition[3].distanceMeters);
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
