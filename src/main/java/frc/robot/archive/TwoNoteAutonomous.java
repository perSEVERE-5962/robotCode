// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAutonomous extends SequentialCommandGroup {
  /** Creates a new PositionThreeTurnMoveBack. */
  public TwoNoteAutonomous(double translationXSupplier, double distanceWanted) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Notification changeLight = Notification.getInstance();
    SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
    addCommands(
        // command to use april tags to align
        new Shoot()
          .onlyIf(() -> changeLight.getNoteState() == Notification.NoteState.NOTE_IN_POSSESSION),
        new TurnToZero(driveTrain, 1),
        new MoveWithDistance(driveTrain, translationXSupplier, 33), // distance from starting point to the outer edge of the note
        new IntakeNote()
          .onlyIf(() -> changeLight.getNoteState() == Notification.NoteState.NOTE_NOT_IN_POSSESSION),
        new Shoot()
          .onlyIf(() -> changeLight.getNoteState() == Notification.NoteState.NOTE_IN_POSSESSION)
    );
  }
}
