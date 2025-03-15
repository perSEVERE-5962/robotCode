// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetFuuctionComplete extends SequentialCommandGroup {
  /** Creates a new ResetFuuctionComplete. */
  public ResetFuuctionComplete() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPivotPosition(4).withTimeout(1),
      new StopIntake()/* .andThen(new SetPivotPosition(4)) */.andThen(new SetPivotPosition(6).withTimeout(1)).andThen(new SetWristPosition(6)).andThen(new SetReachPosition(6))
    );
  }
}
