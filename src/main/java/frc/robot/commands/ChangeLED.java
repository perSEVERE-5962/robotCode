// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Notification;

public class ChangeLED extends Command {
  private final Notification m_notification;
  private boolean noteState;

  public ChangeLED(Notification notification, boolean noteState) {
    m_notification = notification;
    addRequirements(notification);
    this.noteState = noteState;

  }

  @Override
  public void initialize() {
    m_notification.updateState(noteState);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;

  }

}
