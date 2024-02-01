// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Shorter way to express X/Y/Z values */
public class Vec3 {
    public double x = 0;
    public double y = 0;
    public double z = 0;
    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3(double[] pos) {
        this.x = pos[0];
        this.y = pos[1];
        this.z = pos[2];
    }

    public void setAll(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double[] toDoubleArray() {
        return new double[] {this.x, this.y, this.z};
    }

    public double length() {
        return Math.sqrt(x*x + y*y + z*z);
    }

    public void normalize() {
        double length = Math.sqrt(x*x + y*y + z*z);
        if (length == 0) { return; }
        this.x = x/length;
        this.y = y/length;
        this.z = z/length;
    }

    public Vec3 difference(Vec3 source) {
        return new Vec3(this.x - source.x, this.y - source.y, this.z = source.z);
    }
}
