package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
  private final BooleanSupplier fieldOrientedFunction;
  // private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public DriveCommand(
      SwerveSubsystem swerveSubsystem,
      DoubleSupplier xSpdFunction,
      DoubleSupplier ySpdFunction,
      DoubleSupplier turningSpdFunction,
      BooleanSupplier fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    // this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turningLimiter =
    //     new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.getAsDouble();
    double ySpeed = ySpdFunction.getAsDouble();
    double turningSpeed = turningSpdFunction.getAsDouble();

    // 2. Apply deadband
    // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    // ySpeed = Math.abs(ySpeed) > /*OIConstants.kDeadband*/ .4 ? ySpeed : 0.0;
    // turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.4);
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, 0.15);

    // 3. Make the driving smoother
    // xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // turningSpeed =
    //     turningLimiter.calculate(turningSpeed)
    //         * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    // if (fieldOrientedFunction.getAsBoolean()) {
    // Relative to field
    //  chassisSpeeds =
    //      ChassisSpeeds.fromFieldRelativeSpeeds(
    //          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    // } else {
    // Relative to robot
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    // }

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
