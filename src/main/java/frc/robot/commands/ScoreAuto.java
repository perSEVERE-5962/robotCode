// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manipulator.*;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAuto extends SequentialCommandGroup {
  /** Creates a new ScoreAuto. */
  public ScoreAuto() {
    addCommands(
      new ScoreConePositionAutoWithTimeLimit(),
      new MoveRollerAuto(false, 2000),
      new ParallelCommandGroup(new ResetPosition(), new ForwardTime(SwerveSubsystem.getInstance(), 0.5, 3000))
    );
  }
}
