package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommandWithThrottle extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction, throttle;
  private final BooleanSupplier fieldOrientedFunction;
  // private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public DriveCommandWithThrottle(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSpdFunction,
      DoubleSupplier ySpdFunction,
      DoubleSupplier turningSpdFunction,
      BooleanSupplier fieldOrientedFunction,
      DoubleSupplier throttle) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.throttle = throttle;
    // this.xLimiter = new
    // SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.yLimiter = new
    // SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turningLimiter =
    // new
    // SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.getAsDouble() * ((throttle.getAsDouble() + 1) * 0.5);
    double ySpeed = ySpdFunction.getAsDouble() * ((throttle.getAsDouble() + 1) * 0.5);
    double turningSpeed = turningSpdFunction.getAsDouble() * ((throttle.getAsDouble() + 1) * 0.5);

    // 2. Apply deadband
    // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    // ySpeed = Math.abs(ySpeed) > /*OIConstants.kDeadband*/ .4 ? ySpeed : 0.0;
    // turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed
    // : 0.0;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, 0.4); // 0.15 for xbox

    ySpeed *= Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    xSpeed *= Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    turningSpeed *= Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    // 3. Make the driving smoother
    // xSpeed = xLimiter.calculate(xSpeed) *
    // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // ySpeed = yLimiter.calculate(ySpeed) *
    // DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // turningSpeed =
    // turningLimiter.calculate(turningSpeed)
    // * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.getAsBoolean()) {
      // Relative to field
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed * -1, turningSpeed * -1, swerveSubsystem.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed * -1, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
