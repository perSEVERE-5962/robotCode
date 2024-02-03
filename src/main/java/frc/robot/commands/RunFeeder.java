// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.Constants.ColorConstants;
import frc.robot.sensors.UltrasonicAnalog;

public class RunFeeder extends Command {
  private Intake feeder;
  private Notification notification;
  private UltrasonicAnalog feederUltrasonic;
  /** Creates a new Feeder. */
  public RunFeeder(Intake feeder, UltrasonicAnalog feederUltrasonic, Notification notification) {
    this.feederUltrasonic=feederUltrasonic;
    this.feeder=feeder;
    this.notification = notification;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.run(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.run(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double Range_of_feeder=feederUltrasonic.getRange();
    if(Range_of_feeder<=2.3){
      notification.setColor(ColorConstants.BlueHue);

      return true;
    }else{
      return false;
    }
  }
}
