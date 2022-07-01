// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SwerveDriveCommand extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final Joystick m_joystick;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(DriveTrain driveTrain, Joystick joystick) {
    m_driveTrain = driveTrain;
    m_joystick = joystick;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DoubleSupplier translationXSupplier = () -> -modifyAxis(m_joystick.getY()); // Axes are flipped here on purpose
    DoubleSupplier translationYSupplier = () -> -modifyAxis(m_joystick.getX());
    DoubleSupplier rotationSupplier = () -> -modifyAxis(m_joystick.getZ());

    m_driveTrain.swerveDrive(translationXSupplier, translationYSupplier, rotationSupplier);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
