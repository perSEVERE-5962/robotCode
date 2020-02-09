/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class MoveArmToIntake extends CommandBase {

  Arm subsystem;


  /**
   * Creates a new MoveArmCommand.
   */
  public MoveArmToIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = new Arm();
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public double getEncoderValues(){
    return subsystem.getEncoderValues();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.intakePosition();

  }

  // private boolean RunMoveArm() {
  //   return false;
  // }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

