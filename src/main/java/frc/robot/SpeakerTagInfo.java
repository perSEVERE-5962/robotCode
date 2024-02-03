// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.DetectAprilTags;

/** April tag information for the speaker */
public class SpeakerTagInfo {
    public static final int TEAM_COLOR_BLUE = 0;
    public static final int TEAM_COLOR_RED = 1;
    public static int kTeamColor = 0;

    public static final class SpeakerIds {
        // The speaker has 2 april tags from it
        public static final int kBlueSpeakerAprilTag1Id = 0;
        public static final int kBlueSpeakerAprilTag2Id = 8;
        public static final int kRedSpeakerAprilTag1Id = 4;
        public static final int kRedSpeakerAprilTag2Id = 3;
        // Blue: 7 center, 8 side
        // Red: 4 center, 3 side
    }

    public static NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("apriltags");
    public static class tag1Pos {
        public static NetworkTable table = aprilTagTable.getSubTable("tag1pos");
        public static NetworkTableEntry x = table.getEntry("x");
        public static NetworkTableEntry y = table.getEntry("y");
        public static NetworkTableEntry z = table.getEntry("z");
    }

    public static class tag2Pos {
        public static NetworkTable table = aprilTagTable.getSubTable("tag2pos");
        public static NetworkTableEntry x = table.getEntry("x");
        public static NetworkTableEntry y = table.getEntry("y");
        public static NetworkTableEntry z = table.getEntry("z");
    }

    public static class tag1Rot {
        public static NetworkTable table = aprilTagTable.getSubTable("tag1rot");
        public static NetworkTableEntry x = table.getEntry("x");
        public static NetworkTableEntry y = table.getEntry("y");
        public static NetworkTableEntry z = table.getEntry("z");
    }

    public static class tag2Rot {
        public static NetworkTable table = aprilTagTable.getSubTable("tag2rot");
        public static NetworkTableEntry x = table.getEntry("x");
        public static NetworkTableEntry y = table.getEntry("y");
        public static NetworkTableEntry z = table.getEntry("z");
    }

    public static NetworkTableEntry tag1DYawEntry = NetworkTableInstance.getDefault().getEntry("angletotag1");
    public static NetworkTableEntry tag2DYawEntry = NetworkTableInstance.getDefault().getEntry("angletotag2");

    public static void update(int tagId1, int tagId2) {
        int aptag1Index = DetectAprilTags.getAprilTagIndexFromId(tagId1);
        if (aptag1Index != -1) {
            Vec3 aptag1Pos = DetectAprilTags.getAprilTagPos(aptag1Index);
            Vec3 aptag1Rot = DetectAprilTags.getAprilTagRot(aptag1Index);
            if (aptag1Pos != null && aptag1Rot != null) {
                double angleToTag = Math.toDegrees(Math.atan2(aptag1Pos.getX(), aptag1Pos.getZ()));
                SpeakerTagInfo.tag1Pos.x.setDouble(aptag1Pos.getX());
                SpeakerTagInfo.tag1Pos.y.setDouble(aptag1Pos.getY());
                SpeakerTagInfo.tag1Pos.z.setDouble(aptag1Pos.getZ());
                SpeakerTagInfo.tag1Rot.x.setDouble(aptag1Rot.getX());
                SpeakerTagInfo.tag1Rot.y.setDouble(aptag1Rot.getY());
                SpeakerTagInfo.tag1Rot.z.setDouble(aptag1Rot.getZ());
                SpeakerTagInfo.tag1DYawEntry.setDouble(angleToTag);
            }
        }

        int aptag2Index = DetectAprilTags.getAprilTagIndexFromId(tagId2);
        if (aptag2Index != -1) {
            Vec3 aptag2Pos = DetectAprilTags.getAprilTagPos(aptag2Index);
            Vec3 aptag2Rot = DetectAprilTags.getAprilTagRot(aptag2Index);
            if (aptag2Pos != null && aptag2Rot != null) {
                double angleToTag = Math.toDegrees(Math.atan2(aptag2Pos.getX(), aptag2Pos.getZ()));
                SpeakerTagInfo.tag2Pos.x.setDouble(aptag2Pos.getX());
                SpeakerTagInfo.tag2Pos.y.setDouble(aptag2Pos.getY());
                SpeakerTagInfo.tag2Pos.z.setDouble(aptag2Pos.getZ());
                SpeakerTagInfo.tag2Rot.x.setDouble(aptag2Rot.getX());
                SpeakerTagInfo.tag2Rot.y.setDouble(aptag2Rot.getY());
                SpeakerTagInfo.tag2Rot.z.setDouble(aptag2Rot.getZ());
                SpeakerTagInfo.tag2DYawEntry.setDouble(angleToTag);
            }
        }
    }

    private static String tab = "Speaker Tags";
    private static GenericEntry tag1DistEntryMeters = Shuffleboard.getTab(tab).add("Tag 1 M", 0)
        .withPosition(0, 0)
        .getEntry();
    private static GenericEntry tag2DistEntryMeters = Shuffleboard.getTab(tab).add("Tag 2 M", 0)
        .withPosition(1, 0)
        .getEntry();
    private static GenericEntry tag1DistEntryInches = Shuffleboard.getTab(tab).add("Tag 1 In", 0)
        .withPosition(0, 1)
        .getEntry();
    private static GenericEntry tag2DistEntryInches = Shuffleboard.getTab(tab).add("Tag 2 In", 0)
        .withPosition(1, 1)
        .getEntry();

    public static void display() {
        tag1DistEntryMeters.setDouble(tag1Pos.z.getDouble(0));
        tag2DistEntryMeters.setDouble(tag2Pos.z.getDouble(0));
        tag1DistEntryInches.setDouble(Units.metersToInches(tag1Pos.z.getDouble(0)));
        tag2DistEntryInches.setDouble(Units.metersToInches(tag2Pos.z.getDouble(0)));
    }
}
