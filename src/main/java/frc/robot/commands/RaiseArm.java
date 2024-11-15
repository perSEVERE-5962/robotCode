// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class RaiseArm extends Command {
  private Arm arm;

  public RaiseArm(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveToPositionWithPID(Constants.ArmPositions.upperLimit); // raise arm
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // arm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getPosition() <= Constants.ArmPositions.upperLimit;
  }
}