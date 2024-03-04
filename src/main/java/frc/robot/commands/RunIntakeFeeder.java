// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Notification.NoteState;

public class RunIntakeFeeder extends Command {
  private Feeder intakefeeder ;
  private final Notification notification;
  
  /** Creates a new Feeder. */
  public RunIntakeFeeder(Feeder feeder, Notification notification){
    this.intakefeeder = feeder;
    this.notification = notification;   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder, notification);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakefeeder.run(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    notification.updateState(NoteState.NOTE_IN_POSSESSION);
    intakefeeder.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakefeeder.isInRange();
  }
}
