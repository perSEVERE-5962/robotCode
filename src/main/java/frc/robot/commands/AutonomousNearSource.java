// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants.TrajectoryConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousNearSource extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public AutonomousNearSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    BooleanSupplier aprilTagFound = (() -> NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getEntry("command").getString("Default") == "Not found");
    addCommands(
      // Start at the opponent's source
      // Wait some time
      new Timer(6000),
      // Move towards the stage
      new MoveWithTrajectory(TrajectoryConstants.kTrajectoryCommonStart,
                             null,
                             null, new Rotation2d(Units.degreesToRadians(135))).getTrajectoryCommandGroup(),
      // Turn towards the speaker (approximately)
      new ParallelRaceGroup(
        new Move(SwerveSubsystem.getInstance(),
                    0, 0, 0.5)
                    .until(aprilTagFound),
        new Timer(2000)
      ),
      new ShootWithApriltag()
    );
  }
}
