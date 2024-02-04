// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;


public class TurnToZeroo extends Command {
  private double initialYaw;
  private double rotationDirection;
  private double rotationSpeed;
  private SwerveSubsystem driveTrain;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public TurnToZeroo(SwerveSubsystem driveTrain, double rotationSpeed) {
    this.driveTrain = driveTrain;
    this.rotationSpeed = rotationSpeed;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialYaw = driveTrain.getYaw();
    if (initialYaw <= 0) {
      rotationDirection = 1;
    }
    else {
      rotationDirection = -1;
    }
  }

  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;
    // if (fieldOrientedFunction.get()) {
    // // Relative to field
    // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    // xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    // } else {
    // Relative to robot
    chassisSpeeds = new ChassisSpeeds(0, 0,  rotationDirection*rotationSpeed);
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }


  @Override
  public boolean isFinished() {
    if (initialYaw < 0 ) {
      if(driveTrain.getYaw() >= 0) {
        return true;
      }
    }
    else {
      if (driveTrain.getYaw() <=0) {
        return true;
      }
    }
    return false;
  }
}
