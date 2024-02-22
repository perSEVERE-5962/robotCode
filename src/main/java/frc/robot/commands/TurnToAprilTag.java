// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SpeakerTagInfo;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TurnToAprilTag extends Command {
  /** Creates a new TurnToAprilTag. */
  private SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private double angleToReach = 0;
  private double yawOffset = 0;
  private double relativeYaw = 0;
  private double speedMultiplier = 1;
  private double tolerance = 0;
  public TurnToAprilTag(double speedMultiplier, double tolerance) {
    this.speedMultiplier = speedMultiplier;
    this.tolerance = tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleToReach = SpeakerTagInfo.tag1Info.getRot().getDyaw();
    yawOffset = driveTrain.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    relativeYaw = driveTrain.getYaw() - yawOffset;
    double angleDifference = relativeYaw - angleToReach;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, (MathUtil.clamp(angleDifference, -1, 1)) * speedMultiplier);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    driveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(angleToReach, relativeYaw, tolerance);
  }
}
