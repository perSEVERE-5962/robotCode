// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class Move extends CommandBase {
  protected Drivetrain m_driveTrain;
  private double translationXSupplier;
  private double translationYSupplier;
  private double rotationSupplier;
  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public Move(
      Drivetrain driveTrain,
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
//    m_driveTrain.drive(translationXSupplier, translationYSupplier, rotationSupplier);
    m_driveTrain.drive(
        /** Driver Oriented */
        new ChassisSpeeds(
            translationXSupplier,
            translationYSupplier,
            rotationSupplier)
    /** Field Oriented */
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    // translationXPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // translationYPercent * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // rotationPercent *
    // DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    // drivetrain.getRotation()
    // )
    );
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
