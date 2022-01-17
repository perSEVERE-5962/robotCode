package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/** */
public class RunTankDrive extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final Joystick m_joystick;

  public RunTankDrive(DriveTrain subsystem, Joystick joystick) {
    m_driveTrain = subsystem;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called just before this Command runs the first time
  public void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    double x = m_joystick.getRawAxis(5);
    double y = m_joystick.getRawAxis(1);
    m_driveTrain.tankDrive(y, x);
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  public void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  public void interrupted() {
    end();
  }
}
