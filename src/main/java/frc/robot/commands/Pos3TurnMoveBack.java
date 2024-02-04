// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos3TurnMoveBack extends SequentialCommandGroup {
  /** Creates a new PositionThreeTurnMoveBack. */
  public Pos3TurnMoveBack(SwerveSubsystem driveTrain, double rotationSpeed, double degreesWanted, double translationXSupplier, double distanceWanted) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Turn(driveTrain, rotationSpeed, degreesWanted),
      new MoveWithDistance(driveTrain, translationXSupplier, distanceWanted)
    );
  }
}
