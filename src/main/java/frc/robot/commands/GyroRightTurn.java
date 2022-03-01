// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GyroRightTurn extends CommandBase {
  private DriveTrain m_driveTrain;
  private AHRS m_gyro;
  private double m_degrees;

  /** Creates a new GyroTurn. */
  public GyroRightTurn(DriveTrain driveTrain, AHRS gyro, double degrees) {
    m_driveTrain = driveTrain;
    m_gyro = gyro;
    m_degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.tankDrive(0.5, -0.5);
    SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gyro.getAngle() > m_degrees;
  }
}
