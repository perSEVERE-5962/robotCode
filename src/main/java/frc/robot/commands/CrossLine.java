// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.LineDetector;

public class CrossLine extends CommandBase {
  /** Creates a new CrossLine. */
  private LineDetector m_lineDetector;
  // private final String red = "Red Value";
  // private final String blue = "Blue Value";
  public CrossLine() {
    this.m_lineDetector = LineDetector.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("CrossLine", "");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("CrossLine", "Line Found");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lineDetector.Sensing_Color();
  }
}
