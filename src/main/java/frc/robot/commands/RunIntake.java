// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import frc.robot.sensors.UltrasonicAnalog;

public class RunIntake extends Command {
  private UltrasonicAnalog intakeUltrasonic;
  private Intake intake;

  /** Creates a new RunIntake. */
  public RunIntake(Intake intake) {
    this.intake = intake;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   this.intakeUltrasonic = intake.getUltrasonicAnalog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //  intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double range_of_intake = intakeUltrasonic.getRange();
    SmartDashboard.putNumber("Range" , range_of_intake);
    if (range_of_intake <= 9) {
      return true;
    } else {

     return false;
    }
  }
}
