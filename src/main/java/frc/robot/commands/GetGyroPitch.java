// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class GetGyroPitch extends CommandBase {
  AHRS gyro;
  boolean checkClimbing;
  /** Creates a new GetGyroPitch. */
  public GetGyroPitch(AHRS gyro, boolean checkClimbing) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gyro = gyro;
    this.checkClimbing = checkClimbing;
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
    if (checkClimbing) {
      return gyro.getPitch() >= Constants.CLIMBING_PITCH;
    }
    return gyro.getPitch() >= Constants.ENGAGED_PITCH;
  }
}
