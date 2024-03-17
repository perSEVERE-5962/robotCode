// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveToShootDistance extends Command {
  /** Creates a new MoveWithinDistance. */
  private double dist = 0.0;
  private double speed = 0.0;
  private NetworkTableEntry distEntry = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getSubTable("pos").getEntry("z");
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  public MoveToShootDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dist = distEntry.getDouble(0);
    speed = 0;
    if (dist <= Constants.SpeakerConstants.kMinDistance) {
      speed = 0.5; // Move backwards
    } else if (dist >= Constants.SpeakerConstants.kMaxDistance) {
      speed = -0.5; // Move forwards
    }

    speed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speed == 0;
  }
}
