// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Intake;

public class RunShooterFeeder extends Command {
  private Intake shooterfeeder;
  private UltrasonicAnalog feederUltrasonic;

  /** Creates a new Feeder. */
  public RunShooterFeeder(Intake feeder, UltrasonicAnalog feederUltrasonic) {
    this.feederUltrasonic = feederUltrasonic;
    this.shooterfeeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterfeeder.run(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   shooterfeeder.run(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double range_of_feeder = feederUltrasonic.getRange();
    System.out.println("Range=" + range_of_feeder);
    if (range_of_feeder <= 2.3) {
      return false;
    } else {
      return true;
    }
  }
}
