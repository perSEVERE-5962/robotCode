// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {
  private Shooter motors;
  private long start;
  public SpinUpShooter(Shooter motors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motors = motors;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterSpeed = SmartDashboard.getNumber("ShooterSpeed", 0);
    System.out.println(shooterSpeed);
    motors.runShooter(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (motors.getVelocity() >= RobotContainer.getInstance().getTargetShootVelocity()) {
      return true;
    }
    return false;
  }
}
