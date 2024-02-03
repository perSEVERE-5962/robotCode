// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TagInfo;
import frc.robot.Vec3;
import frc.robot.subsystems.DetectAprilTags;

public class UpdateTagInfo extends Command {
  private int tagId1 = 0;
  private int tagId2 = 0;
  public UpdateTagInfo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.kTeamColor == Constants.TEAM_COLOR_RED) {
      tagId1 = Constants.SpeakerConstants.kSpeakerRedAprilTag1Id;
      tagId2 = Constants.SpeakerConstants.kSpeakerRedAprilTag2Id;
    } else {
      tagId1 = Constants.SpeakerConstants.kSpeakerBlueAprilTag1Id;
      tagId2 = Constants.SpeakerConstants.kSpeakerBlueAprilTag2Id;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Find both april tags for more precise calculations
    int aptag1Index = DetectAprilTags.getAprilTagIndexFromId(tagId1);
    if (aptag1Index != -1) {
      Vec3 aptag1Pos = DetectAprilTags.getAprilTagPos(aptag1Index);
      Vec3 aptag1Rot = DetectAprilTags.getAprilTagRot(aptag1Index);
      if (aptag1Pos != null && aptag1Rot != null) {
        double angleToTag = Math.toDegrees(Math.atan2(aptag1Pos.x, aptag1Pos.y));
        TagInfo.tag1Pos.x.setDouble(aptag1Pos.x);
        TagInfo.tag1Pos.y.setDouble(aptag1Pos.y);
        TagInfo.tag1Pos.z.setDouble(aptag1Pos.z);
        TagInfo.tag1Rot.x.setDouble(aptag1Rot.x);
        TagInfo.tag1Rot.y.setDouble(aptag1Rot.y);
        TagInfo.tag1Rot.z.setDouble(aptag1Rot.z);
        TagInfo.tag1DYawEntry.setDouble(angleToTag);
      }
    }

    int aptag2Index = DetectAprilTags.getAprilTagIndexFromId(tagId2);
    if (aptag2Index != -1) {
      Vec3 aptag2Pos = DetectAprilTags.getAprilTagPos(aptag2Index);
      Vec3 aptag2Rot = DetectAprilTags.getAprilTagRot(aptag2Index);
      if (aptag2Pos != null && aptag2Rot != null) {
        double angleToTag = Math.toDegrees(Math.atan2(aptag2Pos.x, aptag2Pos.y));
        TagInfo.tag2Pos.x.setDouble(aptag2Pos.x);
        TagInfo.tag2Pos.y.setDouble(aptag2Pos.y);
        TagInfo.tag2Pos.z.setDouble(aptag2Pos.z);
        TagInfo.tag2Rot.x.setDouble(aptag2Rot.x);
        TagInfo.tag2Rot.y.setDouble(aptag2Rot.y);
        TagInfo.tag2Rot.z.setDouble(aptag2Rot.z);
        TagInfo.tag2DYawEntry.setDouble(angleToTag);
      }
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
