// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DetectAprilTags;

public class FindAprilTagAndMove extends CommandBase {
  double[] pos = {0, 0, 0};
  double[] rot = {0, 0, 0};
  /** Creates a new FindAprilTagAndMove. */
  public FindAprilTagAndMove() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pos = DetectAprilTags.getAprilTagPos();
    double x = pos[0];
    NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry("Move");
    String movement = "";
    int detections = DetectAprilTags.aprilTagsDetected();
    if (detections == 1) {
      if (x < -0.2) {
        movement = "Move left";
      } else if (x > 0.2) {
        movement = "Move right";
      } else {
        movement = "Move forward";
      }
    } else if (detections > 1) {
      movement = "Too many april tags";
    } else if (detections == 0) {
      movement = "April tag not found";
    }
    entry.setString(movement);
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
