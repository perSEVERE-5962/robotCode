// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCollectMiddleNote extends ParallelCommandGroup {
  /** Creates a new AutoonomousMiddleNote. */
  public AutonomousCollectMiddleNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ConditionalCommand(
/*
            // Blue team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(4.0, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP).withTimeout(3).andThen(
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(7.25, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.5, DriveConstants.KPID_TKP)
                ).andThen(new ParallelCommandGroup(
                    new IntakeNote(),
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(8.25, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.3, DriveConstants.KPID_TKP).withTimeout(3))),
*/
            // Blue team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(4.0, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP).withTimeout(3).andThen(new ParallelCommandGroup(
                    new IntakeNote(),
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(7.9, -2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.3, DriveConstants.KPID_TKP))),
            // Red team
            new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(5, 2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP).withTimeout(4).andThen(
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(7, 2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP)
                ).andThen(new ParallelRaceGroup(
                    new IntakeNote(),
                    new MoveToPosition(SwerveSubsystem.getInstance(),
                new Pose2d(9, 2.525,
                    new Rotation2d(Units.degreesToRadians(0))),
                0.1, DriveConstants.KPID_TKP))),
            // Conditional
            () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue));
  }
}
