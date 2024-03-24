// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class TurnToAprilTag extends TurntoAngle {
  /** Creates a new TurnToAprilTag. */
  public TurnToAprilTag() {
    super(SwerveSubsystem.getInstance(), NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("speakertags").getEntry("angletotag").getDouble(0), true);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
