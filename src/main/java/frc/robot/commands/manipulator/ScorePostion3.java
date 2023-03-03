// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScorePostion3 extends SequentialCommandGroup {
  /** Creates a new ScorePostion3. */
  public ScorePostion3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new MoveWrist(5), new MoveLift(0), new MoveReach(0), new MoveWrist(0), new GripperOpen()
        new MoveWrist(Constants.WristConstants.kRaiseSoftLimit),
        new MoveLift(Constants.LiftConstants.kPos3),
        new MoveReach(Constants.ReachConstants.kPos3),
        new MoveWrist(Constants.WristConstants.kLowerSoftLimit),
        new GripperOpen());
  }
}
