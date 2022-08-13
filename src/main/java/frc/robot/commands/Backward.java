// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Backward extends CommandBase {
  private double m_distance;
  private DriveTrain m_driveTrain;

  /**
   * Move backwards the distance specified
   *
   * @param distance - the number of inches to move
   */
  public Backward(double distance) {
    m_distance = distance;
    m_driveTrain = RobotContainer.getInstance().getDriveTrain();
    // Use addRequirements() here to declare subsystem dependencies.''
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.arcadeDrive(0.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopDrive();
    ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isFinished = false;
    isFinished = Math.abs(m_driveTrain.getAverageEncoderDistance()) > m_distance;
    return isFinished;
  }
}
