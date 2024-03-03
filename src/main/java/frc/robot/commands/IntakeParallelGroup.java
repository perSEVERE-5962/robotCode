// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Feeder;
//import frc.robot.sensors.UltrasonicAnalog;
//import frc.robot.commands.RunIntakeFeeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeParallelGroup extends ParallelRaceGroup {
  /** Creates a new IntakeParallelGroup. */
  public IntakeParallelGroup(Intake intake, Notification changeLight, Feeder feeder) {
    addCommands(
        new IntakeSequentialGroup(intake, changeLight),
        new RunIntakeFeeder(feeder, changeLight, Notification.NoteState.NOTE_IN_POSSESSION)
    );
  }
}
