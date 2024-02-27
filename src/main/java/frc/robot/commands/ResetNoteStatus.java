// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Notification.NoteState;

public class ResetNoteStatus extends Command {
  /** Creates a new ResetNoteStatus. */
  private Notification notification;
  public ResetNoteStatus(Notification notification) {
    this.notification = notification;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notification.updateState(NoteState.NOTE_NOT_IN_POSSESSION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
