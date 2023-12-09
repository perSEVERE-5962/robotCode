// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class ResetWheels extends CommandBase {
  /** Creates a new ResetWheels. */
  SwerveSubsystem driveTrain;

  private long initialTime;

  public ResetWheels(SwerveSubsystem driveTrain) {
    addRequirements(driveTrain);

    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(SwerveSubsystem.getInstance().frontLeft.getAbsoluteEncoderAngle());
    driveTrain.setWheelsTo0();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() - initialTime > 1000) {
      driveTrain.resetModuleEncoders();
      return true;
    }
    return false;
  }
}
