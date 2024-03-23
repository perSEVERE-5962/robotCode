// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class LogApriltag extends Command {
  /** Creates a new LogApriltag. */
  public LogApriltag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var speakerTagsNT = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags");
    var speakerTagCommand = speakerTagsNT.getEntry("command");
    var speakerTagDist = speakerTagsNT.getSubTable("pos").getEntry("z");
    var speakerTagCenterX = speakerTagsNT.getEntry("centerx");
    var speakerTagDYaw = speakerTagsNT.getEntry("angletotag");
    System.out.println("[Dist: " + speakerTagDist.getDouble(0) + ", Command: " + speakerTagCommand.getString("Default") + ", Center: " + speakerTagCenterX.getDouble(0) + ", DYaw: " + speakerTagDYaw.getDouble(0) + "]");
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
