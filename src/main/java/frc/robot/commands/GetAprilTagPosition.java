// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DetectAprilTags;

public class GetAprilTagPosition extends CommandBase {
  double[] pos = {0, 0, 0};
  double[] rot = {0, 0, 0};
  /** Creates a new FindAprilTagAndMove. */
  public GetAprilTagPosition() {
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
    NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry("TagMove");
    String movement = "";
    int detections = DetectAprilTags.aprilTagsDetected();
    if (detections == 1) {
      if (x < -0.2) {
        movement = "l"; // Move left
      } else if (x > 0.2) {
        movement = "r"; // Move right
      } else {
        movement = "f"; // Move forwards
      }
    } else if (detections > 1) {
      movement = "m"; // Too many tags found
    } else if (detections == 0) {
      movement = "o"; // No tags found
    } else {
      movement = "e"; // Anything else, somehow
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
