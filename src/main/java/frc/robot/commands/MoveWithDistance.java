// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveWithDistance extends Command {
  private double distanceWanted;
  private SwerveSubsystem driveTrain;
  private double speed;

  // private DoubleSupplier m_translationYSupplier;
  // private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public MoveWithDistance(
      SwerveSubsystem driveTrain, double speed, double distanceWanted) {
    this.speed = speed;
    this.driveTrain = driveTrain;
    this.distanceWanted = distanceWanted;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetDrivePosition();
  }


  @Override
  public void execute() {
    double distance = driveTrain.getAverageDistanceInches();
    double currentSpeed = speed;
    if (distance >= 0.5 * distanceWanted && distance <= 0.25 * distanceWanted ){
      currentSpeed = 0.20 * speed;

    }
    else if (distance >= 0.25 * distanceWanted){
      currentSpeed = 0.10 * speed;
    }


    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(currentSpeed, 0, 0);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (driveTrain.getAverageDistanceInches() >= distanceWanted) {
      return true;
    }
    return false;
  }
}