// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class MoveWithTrajectory extends Command {
  /** Creates a new MoveWithTrajectory. */
  private final SwerveSubsystem swerveSubsystem;
  private final Trajectory trajectory;
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  //protected SwerveSubsystem m_driveTrain;
  private SwerveControllerCommand swerveControllerCommand;

  public MoveWithTrajectory(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.47,  0)),
        new Pose2d(0.94, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    xController = new PIDController(DriveConstants.kPXController, 0, 0);
    yController = new PIDController(DriveConstants.kPYController, 0, 0);
    thetaController = new ProfiledPIDController(
        DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveSubsystem);
  }

  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry
    swerveSubsystem.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    swerveControllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveControllerCommand != null && swerveControllerCommand.isFinished();
  }
}

