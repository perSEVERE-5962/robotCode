// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ReachConstants;
import frc.robot.Constants.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeOnGridPos2 extends SequentialCommandGroup {
  /** Creates a new ScoreConeOnGridPos2. */
  public ScoreConeOnGridPos2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            // raise wrist & lift
            new MoveWrist(WristConstants.kRaiseSoftLimit), new MoveLift(LiftConstants.kPos2)),
        new MoveReach(ReachConstants.kPos2),
        new MoveWrist(WristConstants.kLowerSoftLimit),
        new GripperOpen());
  }
}
