// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  private static final Drivetrain Drivetrain = null;

  /** Creates a new AutoTest. */
  public AutoTest() {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new Move(Drivetrain, 1, 0, 0), // move forward
        new Move(Drivetrain, 0, 0, -1), // turn left
        new Move(Drivetrain, 0, 1, 0), // move right
        new Move(Drivetrain, -1, 0, 0), // move back
        new Move(Drivetrain, 1, -1, 0), // move diagonally forward left
        new Move(Drivetrain, 0, 0, 1) // turn right
        );
  }
}
