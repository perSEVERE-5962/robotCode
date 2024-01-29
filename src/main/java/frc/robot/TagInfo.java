// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** April tag information for the speaker */
public class TagInfo {
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
}
