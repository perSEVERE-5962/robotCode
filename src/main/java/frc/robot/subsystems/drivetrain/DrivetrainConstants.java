// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants;

/** Add your docs here. */
public class DrivetrainConstants {
  public static final SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(
              Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
              -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
}
