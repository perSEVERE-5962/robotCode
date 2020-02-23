/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class SmoothArcadeDrive extends CommandBase {
 private final Drive subsystem;
 public Joystick joystick = new Joystick(0);

 public SmoothArcadeDrive(Drive subsystem) {
  this.subsystem = subsystem;
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(subsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.smoothArcadeDrive(joystick.getRawAxis(4), joystick.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
