// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPosition2 extends SequentialCommandGroup {
  /** Creates a new AutoPosition1. */
  public AutoPosition2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // TODO: Add command(s) to turn to the speaker april tag
        //
        new ResetWheels(SwerveSubsystem.getInstance()),
        new ResetNoteStatus(),
        new Move(SwerveSubsystem.getInstance(), 0, -0.5, 0).withTimeout(0.5),
        new TurntoAngle(SwerveSubsystem.getInstance(), 90, true),
        new IntakeNote() // I doubt this decorator nest will work
              .alongWith(
                  new SpinUpShooter(1.0, 1.0, 10)
                        .raceWith(new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                                                          TrajectoryConstants.kTrajectory1Waypoints,
                                                          TrajectoryConstants.kTrajectory1End).getTrajectoryCommandGroup())),
        new Shoot(),
        new TurntoAngle(SwerveSubsystem.getInstance(), 179.9, true),
        new IntakeNote()
              .alongWith(new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                                                TrajectoryConstants.kTrajectory2Waypoints,
                                                TrajectoryConstants.kTrajectory2End).getTrajectoryCommandGroup()),
        new TurntoAngle(SwerveSubsystem.getInstance(), 179.9, true),
        new SpinUpShooter(1.0, 1.0, 10)
              .raceWith(new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                                              TrajectoryConstants.kTrajectory1Waypoints,
                                              TrajectoryConstants.kTrajectory1End).getTrajectoryCommandGroup()),
        new RunShooterFeeder(Feeder.getInstance(), Notification.getInstance()),
        new StopShooter(Shooter.getInstance()),
        new ResetWheels(SwerveSubsystem.getInstance()));
  }
}