// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Notification.NoteState;

public class RunShooterFeeder extends Command {
  private Feeder shooterfeeder;
  private Notification notification;
  // private UltrasonicAnalog feederUltrasonic;
  private long startTime;
  
  /** Creates a new Feeder. 
   * @param noteNotInPossession */
  public RunShooterFeeder(Feeder feeder, Notification notification) {
    this.shooterfeeder = feeder;
    this.notification = notification; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.feederUltrasonic = shooterfeeder.getUltrasonicAnalog();
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterfeeder.run(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // assume that we have shot the note
    notification.updateState(NoteState.NOTE_NOT_IN_POSSESSION);
    shooterfeeder.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long finish = System.currentTimeMillis();
    long timeElapsed = finish - startTime;
    double secondsPassed = timeElapsed / 1000.0;
    return secondsPassed >= 1;
  }
}

