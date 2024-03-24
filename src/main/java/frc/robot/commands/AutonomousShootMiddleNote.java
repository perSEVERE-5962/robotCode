// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousShootMiddleNote extends SequentialCommandGroup {
  /** Creates a new AutoonomousMiddleNote. */
  public AutonomousShootMiddleNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new ConditionalCommand(
            // Blue team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(5.0, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.4, DriveConstants.KPID_TKP).andThen(new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(2.8, -3.3,
                    new Rotation2d(Units.degreesToRadians(-10))),
                0.4, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(1.0, 1.0, 0).withTimeout(0.25))),

            // Red team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(5.0, 2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.3, DriveConstants.KPID_TKP).andThen(new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(2.8, 3.3,
                    new Rotation2d(Units.degreesToRadians(10))),
                0.3, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(1.0, 1.0, 0).withTimeout(0.25))),

            // Conditional
            () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue),

        new RunShooterFeeder(Feeder.getInstance(), Notification.getInstance()),
        new LogApriltag(),
        new StopShooter(Shooter.getInstance()));
    
  }
}



/*
            // Blue team
             new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(4.0, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP).withTimeout(3).andThen(
                     new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(4.0, -1.45,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP)).andThen(
                     new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(2.0, -1.45,
                    new Rotation2d(Units.degreesToRadians(-30))),
                0.1, DriveConstants.KPID_TKP).alongWith(new SpinUpShooter(1.0, 1.0, 0).withTimeout(10))
                ),
            // Red team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(-9, 0, 
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP).withTimeout(4).andThen(
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(-5, -2.525,
                    new Rotation2d(Units.degreesToRadians(-27))),
                0.1, DriveConstants.KPID_TKP)),
 */