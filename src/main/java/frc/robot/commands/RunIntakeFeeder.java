// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.sensors.UltrasonicAnalog;
public class RunIntakeFeeder extends Command {
  private Feeder intakefeeder ;
  private UltrasonicAnalog feederUltrasonic;
  private UltrasonicAnalog feederUltrasonic2;
  
  /** Creates a new Feeder. */
  public RunIntakeFeeder(Feeder feeder){
    this.intakefeeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {   
   // this.feederUltrasonic = intakefeeder.getUltrasonicAnalog(); 
   // this.feederUltrasonic2 = intakefeeder.getUltrasonicAnalog();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakefeeder.run(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakefeeder.run(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return intakefeeder.isInRange();
     
  }
}
