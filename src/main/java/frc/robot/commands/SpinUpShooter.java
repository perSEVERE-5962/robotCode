// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {
  private Shooter motors = Shooter.getInstance();
  private double topShooterSpeed = 1;
  private double bottomShooterSpeed = 1;
  private double topMotorCurrentRPM = 0;
  private double bottomMotorCurrentRPM = 0;

  private long initialTime = 0;
  private long waitTime = 0;

  /**
   * @param topShooterSpeed Speed of the top motors. Use positive values.
   * @param bottomShooterSpeed Speed of the bottom motors. Use positive values.
   * @param overrideWithWaitTime By default, wait until the motors reach max RPM. Set to a nonzero value to override this behavior with a wait time.
   * <p> It's still recommended to decorate this command with `.withTimeout` even with 0 wait time.
   */
  public SpinUpShooter(double topShooterSpeed, double bottomShooterSpeed, long overrideWithWaitTime) {
    this.topShooterSpeed = topShooterSpeed;
    this.bottomShooterSpeed = bottomShooterSpeed;
    this.waitTime = overrideWithWaitTime;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motors.runTopShooter(topShooterSpeed);
    motors.runBottomShooter(bottomShooterSpeed);

    if (waitTime == 0) {
      topMotorCurrentRPM = motors.getTopVelocity();
      bottomMotorCurrentRPM = motors.getBottomVelocity();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (waitTime == 0) {
      boolean topIsMax = topMotorCurrentRPM >= Constants.ShooterConstants.kMaxMotorRPM * topShooterSpeed;
      boolean bottomIsMax = bottomMotorCurrentRPM >= Constants.ShooterConstants.kMaxMotorRPM * bottomShooterSpeed;
      return topIsMax && bottomIsMax;
    } else {
      long finish = System.currentTimeMillis();
      long timeElapsed = finish - initialTime;
      double secondsPassed = timeElapsed / 1000.0;
      return secondsPassed >= waitTime;
    }
  }
}
