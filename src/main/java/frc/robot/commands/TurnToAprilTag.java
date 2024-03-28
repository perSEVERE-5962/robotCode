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
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TurnToAprilTag extends Command {
  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
  private NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getEntry("angletotag");
  private double goal;
  private HolonomicDriveController holonomicDriveController =
      new HolonomicDriveController(
          new PIDController(DriveConstants.kPID_XKP, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
          new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
          new ProfiledPIDController(DriveConstants.kPID_TKP_tele,DriveConstants.KPID_TKI, DriveConstants.KPID_TKD,
              DriveConstants.kThetaControllerConstraints));
  private Pose2d startPos = new Pose2d();
  private Pose2d targetPose2d = new Pose2d();
  private int finishCounter = 0;

  public TurnToAprilTag() {
      holonomicDriveController.setTolerance(new Pose2d(2, 1.0, Rotation2d.fromDegrees(0.75)));
  }

  @Override
  public void initialize() {
      startPos = swerve.getPose();
      goal = -Rotation2d.fromRadians(entry.getDouble(0)).getDegrees();
      targetPose2d = new Pose2d(startPos.getTranslation(),
              startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
  }

  @Override
  public void execute() {
      Pose2d currPose2d = swerve.getPose();
      ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
          targetPose2d, 0, targetPose2d.getRotation());
      SwerveModuleState[] moduleStates = 
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      swerve.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupt) {
      swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
      if (holonomicDriveController.atReference() || (swerve.isAtAngle(goal))) {
          finishCounter++;
      } else {
          finishCounter = 0;
      }

      return finishCounter > 2;
  }
}
