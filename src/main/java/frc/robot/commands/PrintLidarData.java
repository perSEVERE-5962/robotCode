// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AddToShuffleboard;

public class PrintLidarData extends CommandBase {
  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable table = networkTable.getTable("lidar");
  Boolean isDone = false;
  GenericEntry m_entry;
  /** Creates a new PrintLidarData. */
  public PrintLidarData() {
    m_entry = AddToShuffleboard.add("LiDAR", "Action", "");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table.putValue("action", NetworkTableValue.makeString(""));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String action = table.getEntry("action").toString();
    m_entry.setString(action);
    switch (action) {
      case "left":
        m_entry.setString("Left");
        break;
      case "right":
        m_entry.setString("Right");
        break;
      case "stop":
        m_entry.setString("Stop");
        isDone = true;
        break;
      default:
        m_entry.setString("");
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
