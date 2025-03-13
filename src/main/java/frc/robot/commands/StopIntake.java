// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopIntake extends Command {
  /** Creates a new StopIntake. */
 
  
    private Intake intakeSub;
    
    public StopIntake(){
        intakeSub = Intake.getInstance();
        addRequirements(intakeSub);
        
       }

    @Override
  public void initialize() {}

    @Override
  public void execute() {
    intakeSub.run(0);
    
  
    
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }


}
