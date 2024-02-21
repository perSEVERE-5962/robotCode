// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
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
  private SwerveSubsystem swerveSubsystem;
  private Trajectory trajectory;
  //protected SwerveSubsystem m_driveTrain;
  private SwerveControllerCommand swerveControllerCommand;

  public MoveWithTrajectory(SwerveSubsystem swerveSubsystem) {
    
    
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

     trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.47, 0)),
        new Pose2d(0.94, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(DriveConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
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
