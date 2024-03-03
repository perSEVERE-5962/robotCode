// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.SpeakerTagInfo;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinUpShooter extends Command {
  private Shooter motors;
 // private double shooterSpeed;
  //private double speedPercent= 85;
  private long startTime;
  private double distanceFromTag;
  private double speed=1;
  public SpinUpShooter(Shooter motors, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motors = motors;
    this.speed = speed;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    // distanceFromTag = SpeakerTagInfo.tag1Info.getPos().getZ();
    //  if (distanceFromTag <= Units.inchesToMeters(151) && distanceFromTag > Units.inchesToMeters(132)){
    //  speed=0.7;

    // }else if(distanceFromTag >= Units.inchesToMeters(112) && distanceFromTag <= Units.inchesToMeters(132)){
     speed=1;
    //}

    //SmartDashboard.putBoolean("Shooter at max speed", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //speedPercent = SmartDashboard.getNumber("ShooterSpeed", 0);
    //shooterSpeed = (speedPercent/100);
    motors.runShooter(speed);
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
    long timeElapsed = finish - startTime;
    double secondsPassed = timeElapsed / 1000.0;
    return secondsPassed >= 1;
  }
}
