// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Feeder;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNote extends SequentialCommandGroup {
  private final boolean noteRequired=false ;
  /** Creates a new IntakeNote. */

  public IntakeNote(Intake intake, UltrasonicAnalog intakeUltrasonic, UltrasonicAnalog feederUltrasonic, Notification changeLight, Feeder feeder ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   boolean checkForNote=changeLight.getNoteState() ;
    if (checkForNote == noteRequired) {
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new RunIntake(intake, intakeUltrasonic),
          new ChangeLED(changeLight, true)
        ),
        new RunIntakeFeeder(feeder, feederUltrasonic)
      ),
      new StopIntake(intake)
    );
    }
  }
}
