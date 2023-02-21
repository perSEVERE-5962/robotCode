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
public class GetConeFromDoubleSubstation extends SequentialCommandGroup {
  /** Creates a new GetConeFromDoubleSubstation. */
  public GetConeFromDoubleSubstation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // extend reach & close gripper
        new MoveReach(ReachConstants.kSubStation),
        new GripperClose(),
        new ParallelCommandGroup(
            // raise lift, raise wrist, & retract reach to ensure cone is off platform
            new MoveLift(LiftConstants.kRaiseSoftLimit),
            new MoveWrist(WristConstants.kRaiseSoftLimit),
            new MoveReach(ReachConstants.kRetractSoftLimit)),
        // now that we have cleared the platform, lower the lift
        new MoveLift(LiftConstants.kLowerSoftLimit));
  }
}
