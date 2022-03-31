// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class LowerArm extends CommandBase {
  private Arm m_arm;

  public LowerArm(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_arm.getPosition() < -15) {
    //   m_arm.moveArm(-0.1);
    // } else {
    //   m_arm.moveArm(-0.25);
    // }
    m_arm.moveToPositionWithPID(Constants.ArmPositions.lowerLimit); // lower arm
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_arm.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getPosition() <= Constants.ArmPositions.lowerLimit;
  }
}
