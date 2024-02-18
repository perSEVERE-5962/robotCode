// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {
  private Shooter motors;
 // private double shooterSpeed;
  //private double speedPercent= 85;
  private long start;
  
  public SpinUpShooter(Shooter motors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motors = motors;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = System.currentTimeMillis();
    //SmartDashboard.putBoolean("Shooter at max speed", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //speedPercent = SmartDashboard.getNumber("ShooterSpeed", 0);
    //shooterSpeed = (speedPercent/100);
    motors.runShooter(0.7);
    //SmartDashboard.putNumber("current shooter speed",shooterSpeed);
    //motors.runShooter(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (motors.getVelocity() >= shooterSpeed) {
    //   return true;
    // }
    // return false;
    long finish = System.currentTimeMillis();
    long timeElapsed = finish - start;
    if (timeElapsed / 1000 >= 2) {
      return true;
    }
    return false;
  }
}
