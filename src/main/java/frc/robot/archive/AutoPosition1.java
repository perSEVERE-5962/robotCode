// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.MoveWithTrajectory;
import frc.robot.commands.RunShooterFeeder;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StopShooter;
import frc.robot.commands.TurntoAngle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPosition1 extends ParallelCommandGroup {
      double turnAngle = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 110 : -110;
  /** Creates a new AutoPosition1. */
  public AutoPosition1(SwerveSubsystem swerveSubsystem, Shooter shooter, Feeder feeder, Notification notification,
      Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SpinUpShooter(1.0, 1.0, 10)
              .raceWith(new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                                                TrajectoryConstants.kTrajectory1Waypoints,
                                                TrajectoryConstants.kTrajectory1End).getTrajectoryCommandGroup()),
        new RunShooterFeeder(feeder, notification),
        new StopShooter(shooter),
        new TurntoAngle(swerveSubsystem, turnAngle, true),
        new IntakeNote()
              .alongWith(new MoveToPosition(swerveSubsystem, new Pose2d(0.1, 0.0, new Rotation2d(0)))),
        new SpinUpShooter(1.0, 1.0, 10)
              .raceWith(new TurntoAngle(swerveSubsystem, 0, true)),
        new RunShooterFeeder(feeder, notification),
        new StopShooter(shooter));
  }
}
