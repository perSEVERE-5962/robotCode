// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousToTheOutside extends SequentialCommandGroup {
  /** Creates a new AutonomousToTheOutside. */
  public AutonomousToTheOutside() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ConditionalCommand(
                // Blue team
                new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(0, 0.7,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.1, DriveConstants.KPID_TKP).andThen(new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(3.7, 0.7,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.1, DriveConstants.KPID_TKP).withTimeout(3)).withTimeout(3),

                // Red team
                new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(0, -0.7,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.1, DriveConstants.KPID_TKP).andThen(new MoveToPosition(SwerveSubsystem.getInstance(),
                    new Pose2d(3.7, -0.7,
                        new Rotation2d(Units.degreesToRadians(0))),
                    0.1, DriveConstants.KPID_TKP).withTimeout(3)).withTimeout(3),
                    
                // Conditional()
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            )

    );
  }
}
