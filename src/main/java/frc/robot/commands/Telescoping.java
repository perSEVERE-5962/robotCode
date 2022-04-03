// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.telescopehanger;

public class Telescoping extends CommandBase {
  private  telescopehanger m_telescoping; 
  private  Joystick m_controller;

  /** Creates a new Telescoping. */
  public Telescoping(telescopehanger telescoping, Joystick controller) {
    m_controller = controller;
    m_telescoping = telescoping;
    addRequirements(telescoping);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void addRequirements(Telescoping telescoping) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Telescoping","initalized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRawAxis(3) > 0.1) {
      SmartDashboard.putString("Telescoping","extending");
      m_telescoping.telescopeControl(m_controller.getRawAxis(3));
    } else if (m_controller.getRawAxis(2) > 0.1) {
      SmartDashboard.putString("Telescoping","retracting");
      m_telescoping.telescopeControl(-m_controller.getRawAxis(2));
    } else {
      SmartDashboard.putString("Telescoping","stop");
      m_telescoping.telescopeControl(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Telescoping","end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
