// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPosition2 extends ParallelCommandGroup {
  /** Creates a new AutoPosition1. */
  public AutoPosition2(SwerveSubsystem swerveSubsystem, Shooter shooter, Feeder feeder, Notification changeLight, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Shoot(shooter, feeder, changeLight),
      new TurnToZero(swerveSubsystem, 1),
      new MoveWithTrajectory(swerveSubsystem).getTrajectoryCommandGroup(),
      new IntakeNote(intake, changeLight, feeder),
      new Shoot(shooter, feeder, changeLight)

    );
  }
}