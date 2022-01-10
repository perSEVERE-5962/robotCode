package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class RunTankDrive extends CommandBase {
  private final Drive subsystem;
  public Joystick joystick = new Joystick(0);

  public RunTankDrive(Drive subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called just before this Command runs the first time
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    double x = joystick.getRawAxis(5);
    double y = joystick.getRawAxis(1);
    // x = x*x*x;
    // y = y*y*y;
    subsystem.smoothTankDrive(y, x);
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  public void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
    end();
  }
}