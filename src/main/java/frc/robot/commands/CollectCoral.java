// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class CollectCoral extends Command {
  private Intake intakeSub;
  private Wrist wristSub;
  /** Creates a new PickUpIntake. */
  public CollectCoral() {
    intakeSub = Intake.getInstance();
    wristSub= Wrist.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub,wristSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.run(0.3);
    //intakeSub.moveToPositionWithPID(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSub.checkNormallyOpenLimitSwitch();
  }
}
