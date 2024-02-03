// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ColorConstants;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Notification;

public class RunIntake extends Command {
  private UltrasonicAnalog intakeUltrasonic;
  public Notification notification;
  private Intake intake;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, UltrasonicAnalog intakeUltrasonic, Notification notification) {
    this.intakeUltrasonic = intakeUltrasonic;
    this.intake = intake;
    this.notification = notification;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    notification.setColor(ColorConstants.RedHue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double range_of_intake = intakeUltrasonic.getRange();
    System.out.println("range =" + range_of_intake);
    if (range_of_intake <= 2.3) {
      notification.setColor(ColorConstants.YellowHue);
      return true;
    } else {

      return false;
    }
  }
}
