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
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRawAxis(6) > 0.2) {
      m_hanger.moveToPositionWithPID(Constants.HangerPositions.lowerlimit); // lower position of the hanger
    } else (m_controller.getRawAxis(5) < -0.2) {
      m_hanger.moveToPositionWithPID(Constants.HangerPositions.upperlimit); // upper position of the hanger 
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}