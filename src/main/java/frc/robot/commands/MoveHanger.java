// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hanger;

public class MoveHanger extends CommandBase {
  private static Joystick m_controller;
  private static Hanger m_hanger;

  public MoveHanger(Joystick controller, Hanger hanger) {
    m_controller = controller;
    m_hanger = hanger;
    addRequirements(m_hanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRawButton(6)) {
      m_hanger.moveHanger(-0.25); // lower position of the hanger
    } else if (m_controller.getRawButton(5)) {
      m_hanger.moveHanger(0.25); // upper position of the hanger
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
