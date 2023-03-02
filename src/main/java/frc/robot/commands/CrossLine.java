// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AddToShuffleboard;
// import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.LineDetector;

public class CrossLine extends CommandBase {
  /** Creates a new CrossLine. */
  private LineDetector m_lineDetector;

  Boolean is_line_found = false;
  // private final String red = "Red Value";
  // private final String blue = "Blue Value";
  public CrossLine() {
    this.m_lineDetector = LineDetector.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AddToShuffleboard.add("Line Detector", "Is Line Crossed", is_line_found);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    is_line_found = m_lineDetector.Sensing_Color();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lineDetector.Sensing_Color();
  }
}
