// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Notification;

public class ChangeLED extends Command {
  private final Notification notification;
  private Notification.NoteState noteState;

  public ChangeLED(Notification notification, Notification.NoteState noteState) {
    this.notification = notification;
    this.noteState = noteState;

    addRequirements(notification);
  }

  @Override
  public void initialize() {
    notification.updateState(noteState);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
