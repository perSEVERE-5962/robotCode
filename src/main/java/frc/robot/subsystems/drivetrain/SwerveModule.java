package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private PIDController turningPidController;

  // private final AnalogInput absoluteEncoder;
  private final CANCoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderId,
      double absoluteEncoderOffset,
      boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderId);
    // SAT CHANGE: absoluteEncoder.setPosition(0);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveMotor.setSmartCurrentLimit(40);
    turningMotor.setSmartCurrentLimit(20);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    driveMotor.getPIDController().setP(0.1);
    driveMotor.getPIDController().setI(0);
    driveMotor.getPIDController().setD(0);

    driveMotor.getPIDController().setIZone(0);
    driveMotor.getPIDController().setFF(0);

    driveMotor.getPIDController().setOutputRange(-0.5, 0.5);

    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderAngle() {
    double angle = absoluteEncoder.getPosition();
    // SAT CHANGE: double angle = absoluteEncoder.getPosition()*(360/4096);
    return angle;
  }

  public double getAbsoluteEncoderRad() {
    double angle = Math.toRadians(getAbsoluteEncoderAngle());
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
    // SAT CHANGE: turningEncoder.setPosition(0);
    // SAT CHANGE: absoluteEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(
        turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
  }

  public void moveWithPidInches(double position) {
    driveMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setAbsoluteEncoderPosition(double radians) {
    absoluteEncoder.setPosition(Math.toDegrees(radians));
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
}
