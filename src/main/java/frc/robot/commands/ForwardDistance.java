// Copyright (c) FIRST and other WPILib +.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.drivetrain.Drivetrain;


public class ForwardDistance extends Move {
  private double distanceWanted; 
  //private DoubleSupplier m_translationYSupplier;
  //private DoubleSupplier m_rotationSupplier;
  /** Creates a new Forward. */
  public ForwardDistance(Drivetrain driveTrain, double translationXSupplier , double distanceWanted) {
    super(driveTrain, translationXSupplier, 0, 0);
    this.distanceWanted = distanceWanted;
    
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceWanted > m_driveTrain.getAverageEncoder())
      return true;

    return false;
  }
}
