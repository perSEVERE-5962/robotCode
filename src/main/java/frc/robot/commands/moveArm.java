// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class moveArm extends CommandBase {
  /** Creates a new moveArm. */
  private static Joystick m_controller;

  private static Arm m_arm;

  public moveArm(Joystick controller, Arm arm) {
    m_controller = controller;
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRawAxis(5) > 0.2) {
      m_arm.moveArm(0.25);
    } else if (m_controller.getRawAxis(5) < -0.2) {
      m_arm.moveArm(-0.1);
    } else {
      m_arm.moveArm(0);
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
