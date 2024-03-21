// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SpeakerConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveToShootDistance extends Command {
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private Pose2d pose2d;
  private NetworkTableEntry distEntry = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getSubTable("pos").getEntry("z");

  private HolonomicDriveController holonomicDriveController =
  new HolonomicDriveController(
      new PIDController(DriveConstants.kPID_XKP, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
      new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
      new ProfiledPIDController(DriveConstants.KPID_TKP,DriveConstants.KPID_TKI, DriveConstants.KPID_TKD,
          DriveConstants.kThetaControllerConstraints));

  /** Creates a new MoveToShootDistance. */
  public MoveToShootDistance() {
    holonomicDriveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1)));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double dist = distEntry.getDouble(-1);
    if (dist > SpeakerConstants.kMaxDistance) {
      pose2d = new Pose2d(dist - SpeakerConstants.kMaxDistance, 0, new Rotation2d(0));
    } else if (dist < SpeakerConstants.kMinDistance) {
      pose2d = new Pose2d(SpeakerConstants.kMinDistance - dist, 0, new Rotation2d(0));
    } else {
      pose2d = new Pose2d(0, 0, new Rotation2d(0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds =
        // Not great but the y speed needs to be inverted
        holonomicDriveController.calculate(driveTrain.getPose(), pose2d, 0, pose2d.getRotation());
    SwerveModuleState[] moduleStates = 
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holonomicDriveController.atReference();
  }
}
