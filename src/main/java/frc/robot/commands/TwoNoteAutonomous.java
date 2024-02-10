// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAutonomous extends SequentialCommandGroup {
  /** Creates a new PositionThreeTurnMoveBack. */
  public TwoNoteAutonomous(Shooter shooter, Intake feeder, UltrasonicAnalog feederSensor, Notification changeLight, SwerveSubsystem driveTrain, double rotationSpeed, double degreesWanted, double translationXSupplier, double distanceWanted, boolean checkForNote) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // command to use april tags to align
      new Shoot(shooter, feeder, feederSensor, changeLight),
      new TurnToZero(driveTrain, 1),
      new MoveWithDistance(driveTrain, translationXSupplier, 33), // distance from starting point to the outer edge of the note
      new IntakeNote(feeder, feederSensor, feederSensor, changeLight, feeder),
      new Shoot(shooter, feeder, feederSensor, changeLight)    
    );
  }
}
