// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveWithTrajectory {
  /** Creates a new MoveWithTrajectory. */
  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private Trajectory trajectory;
  //protected SwerveSubsystem m_driveTrain;
  private SwerveControllerCommand swerveControllerCommand;

    /*
     * new Pose2d(0, 0, new Rotation2d(0)),
     * List.of(
     *     new Translation2d(0.86, 0)),
     * new Pose2d(2.05, 0, Rotation2d.fromDegrees(0)),
     */

    /*
     * new Pose2d(0.0, 0.0, new Rotation2d(0)),
     * List.of(
     *     new Translation2d(0.85, 0)),
     * new Pose2d(1.74, 0.0, Rotation2d.fromDegrees(0)),
     */

  public MoveWithTrajectory(Pose2d startPoint, List<Translation2d> waypoints, Pose2d endPoint) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    trajectory = TrajectoryGenerator.generateTrajectory(
      startPoint, waypoints, endPoint,
      trajectoryConfig);

    HolonomicDriveController holonomicDriveController =
        new HolonomicDriveController(
          new PIDController(DriveConstants.kPID_XKP, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
          new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
          new ProfiledPIDController(
            DriveConstants.KPID_TKP,
            DriveConstants.KPID_TKI, 
            DriveConstants.KPID_TKD,
            DriveConstants.kThetaControllerConstraints));

    swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);
    
  }

  public MoveWithTrajectory(Pose2d startPoint, List<Translation2d> waypoints, Pose2d endPoint, Rotation2d desiredAngle) {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    trajectory = TrajectoryGenerator.generateTrajectory(
      startPoint, waypoints, endPoint,
      trajectoryConfig);

    HolonomicDriveController holonomicDriveController =
        new HolonomicDriveController(
          new PIDController(DriveConstants.kPID_XKP, DriveConstants.kPID_XKI, DriveConstants.kPID_XKD), 
          new PIDController(DriveConstants.kPID_YKP, DriveConstants.kPID_YKI, DriveConstants.kPID_YKD),
          new ProfiledPIDController(
            DriveConstants.KPID_TKP,
            DriveConstants.KPID_TKI, 
            DriveConstants.KPID_TKD,
            DriveConstants.kThetaControllerConstraints));

    swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        (() -> desiredAngle),
        swerveSubsystem::setModuleStates,
        swerveSubsystem);
    
  }

  public SequentialCommandGroup getTrajectoryCommandGroup() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }   
}
