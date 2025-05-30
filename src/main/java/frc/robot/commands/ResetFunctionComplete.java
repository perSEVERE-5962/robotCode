// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Reach;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetFunctionComplete extends SequentialCommandGroup {
  /** Creates a new ResetFuuctionComplete. */
  public ResetFunctionComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     // new SetWristPosition(10), new SetPivotPosition(10),

      // prior to Mayhem:
      //new StopIntake() .andThen(new SetWristPosition(Constants.ScoringConstants.kStartPos)).andThen(new SetPivotPosition(Constants.ScoringConstants.kStartPos)).andThen(new SetReachPosition(Constants.ScoringConstants.kStartPos))
      // changed at Mayhem:
      new StopIntake()/*.andThen(new SetPivotPosition(Constants.ScoringConstants.kStartPos))*/.andThen(new SetWristPosition(Constants.ScoringConstants.kStartPos)).andThen(new SetReachPosition(Constants.ScoringConstants.kStartPos))    );
  }
}
