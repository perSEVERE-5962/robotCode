// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPosition3 extends ParallelCommandGroup {
  /** Creates a new AutoPosition1. */
  public AutoPosition3(SwerveSubsystem swerveSubsystem, Shooter shooter, Feeder feeder, Notification notification, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SpinUpShooter(shooter, 0),
        new MoveWithTrajectory(swerveSubsystem).getTrajectoryCommandGroup()
        
      ),
      new RunShooterFeeder(feeder, notification),
      new StopShooter(shooter),
      new TurntoAngle(swerveSubsystem, -110.0,true),
      new ParallelCommandGroup(
          new IntakeNote(),
          //new MoveWithDistance(swerveSubsystem, 1, 0.1) //placeholder, try MoveWithPosition instead --> 
          new MoveToPosition(swerveSubsystem, new Pose2d(0.1,0.0,new Rotation2d(0)))
      ),
      new ParallelCommandGroup(
        new SpinUpShooter(shooter, 0),
        new TurntoAngle(swerveSubsystem, 0, true)  // change this because this is position
      ),
      new RunShooterFeeder(feeder, notification),
      new StopShooter(shooter)
    );
  }
}
