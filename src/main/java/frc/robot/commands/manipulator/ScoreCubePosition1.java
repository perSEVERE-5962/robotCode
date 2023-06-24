// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.obsolete.commands.manipulator.CubeGripperClose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCubePosition1 extends SequentialCommandGroup {
  /** Creates a new ScoreCubePosition1. */
  public ScoreCubePosition1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new CubeGripperClose(),
        new MoveWrist(Constants.WristConstants.kRaiseSoftLimit),
        new MoveLift(Constants.LiftConstants.kPos1),
        new MoveReach(Constants.ReachConstants.kPos1),
        new MoveWrist(Constants.WristConstants.kLowerSoftLimit)
        // new CubeGripperOpen()
        );
  }
}
