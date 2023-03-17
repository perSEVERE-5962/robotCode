// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.obsolete;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class IfLevel extends CommandBase {
  SwerveSubsystem m_driveTrain;

  /** Creates a new IfLevel. */
  public IfLevel(SwerveSubsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    // addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getPitch() <= Constants.PITCH_LEVEL - Constants.PITCH_OFFSET;
  }
}