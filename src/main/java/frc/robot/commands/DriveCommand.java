// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

  private final Drivetrain m_driveTrain;
  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  /** Creates a new SwerveDriveCommand. */
  public DriveCommand(Drivetrain driveTrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    m_driveTrain = driveTrain;

    m_translationXSupplier = translationXSupplier;
    m_translationYSupplier = translationYSupplier;
    m_rotationSupplier = rotationSupplier;

    addRequirements(driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.drive(
        /** Driver Oriented */
        new ChassisSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble())
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
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_driveTrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

}
