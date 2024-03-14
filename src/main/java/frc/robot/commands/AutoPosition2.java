// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPosition2 extends SequentialCommandGroup {
  /** Creates a new AutoPosition1. */
  public AutoPosition2(SwerveSubsystem swerveSubsystem, Shooter shooter, Feeder feeder, Notification changeLight, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // TODO: Add command(s) to turn to the speaker april tag
      //
      new ResetWheels(swerveSubsystem),
      new ResetNoteStatus(changeLight),
      new Move(swerveSubsystem, 0, -0.5, 0).withTimeout(0.5),
      new TurntoAngle(swerveSubsystem, 90, true),
      new ParallelCommandGroup(
        new IntakeNote(),
        new SpinUpShooter(shooter, 0),
        new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                              TrajectoryConstants.kTrajectory1Waypoints,
                              TrajectoryConstants.kTrajectory1End).getTrajectoryCommandGroup()
      ),
    
      new Shoot(),
//      new ResetWheels(swerveSubsystem),
      new TurntoAngle(swerveSubsystem, 179.9,true),
//      new ResetWheels(swerveSubsystem),
      new ParallelCommandGroup(
          new IntakeNote(),
          new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                              TrajectoryConstants.kTrajectory2Waypoints,
                              TrajectoryConstants.kTrajectory2End).getTrajectoryCommandGroup()
      ),
//      new ResetWheels(swerveSubsystem),
      new TurntoAngle(swerveSubsystem, 179.9,true),
//      new ResetWheels(swerveSubsystem),
      new ParallelCommandGroup(
        new SpinUpShooter(shooter, 0),
        new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                              TrajectoryConstants.kTrajectory1Waypoints,
                              TrajectoryConstants.kTrajectory1End).getTrajectoryCommandGroup()
      ),
      new RunShooterFeeder(feeder, changeLight),
      new StopShooter(shooter),
      new ResetWheels(swerveSubsystem) 
      // ),
      




      // new Turn(swerveSubsystem, 1, 180),
      // new MoveWithTrajectory(swerveSubsystem).getTrajectoryCommandGroup(),
      // new Shoot(shooter, feeder, changeLight)


    );
  }
}