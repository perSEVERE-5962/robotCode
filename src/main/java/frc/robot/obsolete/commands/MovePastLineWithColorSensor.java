// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.obsolete.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CrossLine;
import frc.robot.commands.ForwardDistance;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MovePastLineWithColorSensor extends SequentialCommandGroup {
  /** Creates a new GroupSeqCom_MovePastLine. */
  public MovePastLineWithColorSensor(SwerveSubsystem driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelRaceGroup(new ForwardDistance(driveTrain, 0.5, 0.5), new CrossLine()),
        new ForwardDistance(driveTrain, 0.5, 0.5 /* Unknown */));
  }
}
