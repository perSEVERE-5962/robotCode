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
  private Pose2d startPos;
  private Pose2d targetPos;
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
    startPos = driveTrain.getPose();
    double dist = distEntry.getDouble(-1);
    if (dist > SpeakerConstants.kMaxDistance) {
      targetPos = new Pose2d(startPos.getX() - (dist - SpeakerConstants.kMaxDistance), startPos.getY(), startPos.getRotation());
    } else if (dist < SpeakerConstants.kMinDistance) {
      targetPos = new Pose2d(startPos.getX() + (SpeakerConstants.kMinDistance - dist), startPos.getY(), startPos.getRotation());
    } else {
      targetPos = new Pose2d(startPos.getTranslation(), startPos.getRotation());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose2d = driveTrain.getPose();
    ChassisSpeeds chassisSpeeds =
        holonomicDriveController.calculate(currPose2d, targetPos, 0, targetPos.getRotation());
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
