// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BLUE_MoveToPositions extends SequentialCommandGroup {
  /** Creates a new MoveToPositions. */
  public BLUE_MoveToPositions(SwerveSubsystem driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ForwardDistance(driveTrain, -0.5, 66),
        new SidewaysDistance(driveTrain, -0.5, 33),
        // new Turn(driveTrain, 0.5, 175),
        // new MoveWrist(Constants.WristConstants.kFloor),
        // new MoveOntoChargingStation(driveTrain),
        new ForwardDistance(driveTrain, 1, 20),
        new ForwardWithGyro(driveTrain, 0.4),
        new Turn(driveTrain, 0.5, 181));
  }
}
