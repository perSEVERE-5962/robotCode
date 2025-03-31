// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
public class ScoreCoral extends Command {
  private Intake intakeSub;
  /** Creates a new ShootWithIntake. */
  private double speed=-0.5;
  public ScoreCoral(double speed) {
    intakeSub = Intake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
    this.speed=speed;

  }
  public ScoreCoral() {
    intakeSub = Intake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.run(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//!intakeSub.isInRange();
  }
}
