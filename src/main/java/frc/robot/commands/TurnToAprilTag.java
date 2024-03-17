// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  private NetworkTableEntry turnCommand = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getEntry("command");
  private boolean isCentered = false;
  private ProfiledPIDController controller = new ProfiledPIDController(DriveConstants.KPID_TKP, DriveConstants.KPID_TKI, DriveConstants.KPID_TKD, DriveConstants.kThetaControllerConstraints);

  public TurnToAprilTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = 0.0;
    String command = turnCommand.getString("Not found");
    if (!command.equals("Not found")) {
      // Turn towards the apriltag
      if (command.equals("Left")) {
        turnSpeed = 0.5;
        isCentered = false;
      } else if (command.equals("Right")) {
        turnSpeed = -0.5;
        isCentered = false;
      } else if (command.equals("Center")) {
        turnSpeed = 0.0;
        isCentered = true;
      }
    } else {
      // Just turn
      turnSpeed = 0.5;
      isCentered = false;
    }

    turnSpeed *= DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turnSpeed);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending");
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isCentered; //|| MathUtil.isNear(backupAngle, driveTrain.getYaw(), backupAngleTolarance);
  }
}
