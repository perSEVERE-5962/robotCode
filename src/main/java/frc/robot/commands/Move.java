// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class Move extends CommandBase {
  protected SwerveSubsystem m_driveTrain;
  private double translationXSupplier;
  private double translationYSupplier;
  private double rotationSupplier;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public Move(
      SwerveSubsystem driveTrain,
      double translationXSupplier,
      double translationYSupplier,
      double rotationSupplier) {
    m_driveTrain = driveTrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    // if (fieldOrientedFunction.get()) {
    // // Relative to field
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    // xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    // } else {
    // Relative to robot
    chassisSpeeds = new ChassisSpeeds(translationXSupplier, translationYSupplier, rotationSupplier);
    // }

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    m_driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  protected void SetRotationSupplier(float value)
  {
    this.rotationSupplier = value;
  }
}
