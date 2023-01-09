// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {

  private final DriveTrain m_driveTrain;
  private final XboxController m_joystick;
  private DoubleSupplier m_translationXSupplier;
  private DoubleSupplier m_translationYSupplier;
  private DoubleSupplier m_rotationSupplier;

  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(DriveTrain driveTrain, XboxController joystick) {
    m_joystick = joystick;
    m_driveTrain = driveTrain;

    m_translationXSupplier =
        () -> -modifyAxis(m_joystick.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    m_translationYSupplier =
        () -> -modifyAxis(m_joystick.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    m_rotationSupplier =
        () ->
            -modifyAxis(m_joystick.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain
        .getDriveInterface()
        .swerveDrive(m_translationXSupplier, m_translationYSupplier, m_rotationSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.getDriveInterface().stopDrive();
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
