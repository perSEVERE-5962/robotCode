// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveBackward extends CommandBase {
  private double m_position;
  private DriveTrain m_driveTrain;
  /** Creates a new AutoDriveForward. */
  public AutoDriveBackward(double position, DriveTrain driveTrain) {
    m_position = position;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.''
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.tankDrive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    isFinished = m_driveTrain.getAverageEncoderDistance() > m_position;
    return isFinished;
  }
}
