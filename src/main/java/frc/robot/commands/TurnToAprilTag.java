// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TurnToAprilTag extends Command {
  /** Creates a new TurnToAprilTag. */
  SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private double turnSpeed = 0;
  private NetworkTableEntry turnCommand = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getEntry("command");
  private boolean isCentered = false;
  public TurnToAprilTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isCentered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (turnCommand.getString("") == "Left") {
      turnSpeed = -0.5;
    } else if (turnCommand.getString("") == "Right") {
      turnSpeed = 0.5;
    } else {
      isCentered = true; // This can also trigger if no apriltags are found
      turnSpeed = 0;
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turnSpeed);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCentered;
  }
}
