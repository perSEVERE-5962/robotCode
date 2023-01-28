// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Move extends CommandBase {
  protected Drivetrain m_driveTrain;
  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;
  private DoubleSupplier rotationSupplier;
  //private DoubleSupplier m_translationYSupplier;
  //private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public Move(Drivetrain driveTrain, double translationXSupplier, double translationYSupplier, double rotationSupplier) {
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.swerveDrive(translationXSupplier, translationYSupplier, rotationSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
