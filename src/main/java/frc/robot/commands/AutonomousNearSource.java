// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousNearSource extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public AutonomousNearSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Start at the opponent's source
      // Wait some time
      new Timer(100), // Wait 6000 for other teams
      // Move towards the stage
      // 2.4, -2.0, -27 -> is blue team
      // 2.8, 2.3, 27 -> is red team
      new ParallelRaceGroup(
        new ConditionalCommand(
          // Blue team
          new MoveToPosition(SwerveSubsystem.getInstance(),
          new Pose2d(2.4, -2.0,
              new Rotation2d(Units.degreesToRadians(-27))
          ),
          0.1, DriveConstants.KPID_TKP).withTimeout(4),
          // Red team
          new MoveToPosition(SwerveSubsystem.getInstance(),
          new Pose2d(2.8, 2.3,
              new Rotation2d(Units.degreesToRadians(27))
          ),
          0.1, DriveConstants.KPID_TKP).withTimeout(4), 
          // Conditional
          () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Blue),
          new SpinUpShooter(1.0, 1.0, 0).withTimeout(10)
      ),
      
      new RunShooterFeeder(Feeder.getInstance(), Notification.getInstance()),
      new StopShooter(Shooter.getInstance())
    );
  }
}
