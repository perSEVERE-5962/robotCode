// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeSpeed extends CommandBase {
  /** Creates a new IntakeSpeed. */
  private static Intake m_armIntake;

  // private static Arm m_arm;
  private static Joystick m_controller;

  public IntakeSpeed(Intake armIntake, Joystick controller /*, Arm arm*/) {
    m_armIntake = armIntake;
    addRequirements(armIntake /*, arm*/);
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_armIntake.armIntake(m_controller.getRawAxis(3));
    if (m_controller.getRawAxis(3) > 0.1) {
      // double position = m_arm.getPosition();
      // if (position <= Constants.ArmPositions.shootmax
      //     && position >= Constants.ArmPositions.shootmin) {
      //    m_armIntake.armIntake(m_controller.getRawAxis(3));
      //  } else {
      //    m_armIntake.armIntake(0);
      //  }
      m_armIntake.armIntake(m_controller.getRawAxis(3));
    } else if (m_controller.getRawAxis(2) > 0.1) {
      m_armIntake.armIntake(-m_controller.getRawAxis(2));
    } else {
      m_armIntake.armIntake(0);
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
