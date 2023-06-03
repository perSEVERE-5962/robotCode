// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * Shorter and more efficent way to control 3 variables, usually when dealing with 3D space.
 */
public class Vec3f {
  float[] pos = new float[3];

  public Vec3f(float x, float y, float z) {
    this.pos[0] = x;
    this.pos[1] = y;
    this.pos[2] = z;
  }

  public Vec3f(float[] pos) {
    this.pos[0] = pos[0];
    this.pos[1] = pos[1];
    this.pos[2] = pos[2];
  }

  public static Vec3f convertToVec3f(float x, float y, float z) {
    return new Vec3f(x, y, z);
  }

  public static Vec3f convertToVec3f(double x, double y, double z) {
    return new Vec3f((float) x, (float) y, (float) z);
  }

  public static Vec3f convertToVec3f(float[] pos) {
    return convertToVec3f(pos[0], pos[1], pos[2]);
  }

  public static Vec3f convertToVec3f(double[] pos) {
    return convertToVec3f((float) pos[0], (float) pos[1], (float) pos[2]);
  }

  /** Takes in another Vec3f's positions and stores it in this one. */
  public void set(Vec3f source) {
    this.pos[0] = source.pos[0];
    this.pos[1] = source.pos[1];
    this.pos[2] = source.pos[2];
  }

  /** Takes in two Vec3f variables and sets the source to the destination. */
  public static void set(Vec3f source, Vec3f destination) {
    destination.set(source);
  }

  public void add(Vec3f source) {
    this.pos[0] += source.pos[0];
    this.pos[1] += source.pos[1];
    this.pos[2] += source.pos[2];
  }

  // Gets the linear distance between the Vec3f source point and the destination.
  public static double calcLinearDistance(Vec3f source, Vec3f destination) {
    double distance =
        Math.sqrt(
            Math.pow((double) (source.pos[0] - destination.pos[0]), 2)
                + Math.pow((double) (source.pos[1] - destination.pos[1]), 2)
                + Math.pow((double) (source.pos[2] - destination.pos[2]), 2));
    return distance;
  }
}
