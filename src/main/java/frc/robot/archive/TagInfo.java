// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** April tag information for the speaker */
public class TagInfo {
    public static NetworkTable aprilTagTable = NetworkTableInstance.getDefault().getTable("apriltags");

    private NetworkTable tagTable;

    public class TagPosition {
        private NetworkTable tagPosTable;
        private NetworkTableEntry entryX;
        private NetworkTableEntry entryY;
        private NetworkTableEntry entryZ;

        private TagPosition(NetworkTable table) {
            this.tagPosTable = table.getSubTable("pos");
            this.entryX = tagPosTable.getEntry("x");
            this.entryY = tagPosTable.getEntry("y");
            this.entryZ = tagPosTable.getEntry("z");
        }

        public NetworkTable getPosTable() {
            return this.tagPosTable;
        };

        /** Left/right */
        public double getX() {
            return this.entryX.getDouble(0);
        }
        /** Height */
        public double getY() {
            return this.entryY.getDouble(0);
        }
        /** Distance */
        public double getZ() {
            return this.entryZ.getDouble(0);
        }

        public Vec3 getAll() {
            return new Vec3(this.entryX.getDouble(0), this.entryY.getDouble(0), this.entryZ.getDouble(0));
        }
    }
    private TagPosition tagPosEntries;

    public class TagRotation {
        private NetworkTable tagRotTable;
        private NetworkTableEntry entryX;
        private NetworkTableEntry entryY;
        private NetworkTableEntry entryZ;

        private NetworkTableEntry dyaw;

        private TagRotation(NetworkTable table) {
            this.tagRotTable = table.getSubTable("rot");
            this.entryX = tagRotTable.getEntry("x");
            this.entryY = tagRotTable.getEntry("y");
            this.entryZ = tagRotTable.getEntry("z");
            this.dyaw = tagRotTable.getEntry("dyaw");
        }

        public NetworkTable getRotTable() {
            return this.tagRotTable;
        };

        /** Pitch */
        public double getX() {
            return this.entryX.getDouble(0);
        }
        /** Yaw */
        public double getY() {
            return this.entryY.getDouble(0);
        }
        /** Roll */
        public double getZ() {
            return this.entryZ.getDouble(0);
        }

        /** Positive = left; Negative = right */
        public double getDyaw() {
            return this.dyaw.getDouble(0);
        }

        public Vec3 getAll() {
            return new Vec3(this.entryX.getDouble(0), this.entryY.getDouble(0), this.entryZ.getDouble(0));
        }
    }
    private TagRotation tagRotEntries;

    private int tagId = -1;
    private boolean hasEntry = false;

    public TagInfo(int tagId) {
        this.tagTable = NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("tag" + tagId);
        this.tagPosEntries = new TagPosition(this.tagTable);
        this.tagRotEntries = new TagRotation(this.tagTable);
        this.tagId = tagId;
    }

    public int getId() {
        return this.tagId;
    }

    public TagPosition getPos() {
        return tagPosEntries;
    }

    public TagRotation getRot() {
        return tagRotEntries;
    }

    public void updateShuffleboard() {
        if (!this.hasEntry) {
            String tab = "April Tag Info";
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " pos X", () -> getPos().getX());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " pos Y", () -> getPos().getY());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " pos Z", () -> getPos().getZ());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " rot X", () -> getRot().getX());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " rot Y", () -> getRot().getY());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " rot Z", () -> getRot().getZ());
            Shuffleboard.getTab(tab).addDouble("Tag " + this.tagId + " dyaw", () -> getRot().getDyaw());
            this.hasEntry = true;
        }
    }

    public void update() {
        int id = this.tagId;
        if (id != -1) {
            int index = DetectAprilTags.getAprilTagIndexFromId(id);
            if (index != -1) {
                Vec3 tagPos = DetectAprilTags.getAprilTagPos(index);
                Vec3 tagRot = DetectAprilTags.getAprilTagRot(index);
                if (tagPos != null && tagRot != null) {
                    double angleToTag = Math.toDegrees(Math.atan2(tagPos.getX(), tagPos.getZ()));
                    tagPosEntries.entryX.setDouble(tagPos.getX());
                    tagPosEntries.entryY.setDouble(tagPos.getY());
                    tagPosEntries.entryZ.setDouble(tagPos.getZ());
                    tagRotEntries.entryX.setDouble(tagRot.getX());
                    tagRotEntries.entryY.setDouble(tagRot.getY());
                    tagRotEntries.entryZ.setDouble(tagRot.getZ());
                    tagRotEntries.dyaw.setDouble(angleToTag);
                }
            }
        }
    }
}