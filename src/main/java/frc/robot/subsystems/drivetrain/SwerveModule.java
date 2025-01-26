package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.SDSModules.SDSModuleInterface;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor; 
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; 
import com.revrobotics.spark.config.SparkMaxConfig; 
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode; 
import com.revrobotics.spark.SparkBase.ResetMode; 
import com.revrobotics.spark.SparkLowLevel.MotorType; 

public class SwerveModule {

  private final SparkMax driveMotor; 
  private final SparkMax turningMotor; 

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private PIDController turningPidController;
  private PIDController resetController;

  // private final AnalogInput absoluteEncoder;
  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private double absoluteEncoderOffsetRad;

  private SparkMaxConfig motorConfig; 
  private SparkMaxConfig turningConfig;   
  private SDSModuleInterface sdsModuleInterface;

  public SwerveModule(
      int driveMotorId,
      int turningMotorId,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      int absoluteEncoderId,
      double absoluteEncoderOffset,
      boolean absoluteEncoderReversed,
      SDSModuleInterface sdsModuleInterface) {

    this.sdsModuleInterface = sdsModuleInterface;
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId, Constants.DriveConstants.kCanBusName);
    // SAT CHANGE: absoluteEncoder.setPosition(0);
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    absoluteEncoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    absoluteEncoder.getPosition().setUpdateFrequency(50);
    absoluteEncoder.getVelocity().setUpdateFrequency(50);

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless); 
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless); 

    motorConfig = new SparkMaxConfig(); 
    turningConfig = new SparkMaxConfig(); 
 
 
    motorConfig
      .idleMode(IdleMode.kBrake) 
      .inverted(driveMotorReversed) 
      .smartCurrentLimit(40); 
 
 
    turningConfig
      .idleMode(IdleMode.kBrake) 
      .inverted(turningMotorReversed) 
      .smartCurrentLimit(20); 
 
 
    motorConfig.encoder 
        .positionConversionFactor(sdsModuleInterface.getDriveEncoderRot2Meter()) 
        .velocityConversionFactor(sdsModuleInterface.getDriveEncoderRPM2MeterPerSec()); 
    turningConfig.encoder 
        .positionConversionFactor(sdsModuleInterface.getTurningEncoderRot2Rad()) 
        .velocityConversionFactor(sdsModuleInterface.getTurningEncoderRPM2RadPerSec()); 
 
    motorConfig.closedLoop 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder) 
        .p(0.1) 
        .i(0) 
        .d(0) 
        .outputRange(-0.5,0.5) 
        .velocityFF(0) 
        .iZone(0); 

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    turningPidController = new PIDController(sdsModuleInterface.getPTurning(), 0, 0); 

    resetController = new PIDController(0.01, 0, 0);

    resetEncoders();
  }

  public void setOffsets(double value) {
    this.absoluteEncoderOffsetRad = value;
  }

  public double getOffsets() {
    return this.absoluteEncoderOffsetRad;
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
    double angle = Rotation2d.fromRotations(absoluteEncoder.getPosition().getValueAsDouble()).getDegrees();
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

  public void resetEncodersWithOffsets() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(absoluteEncoderOffsetRad * (absoluteEncoderReversed ? -1.0 : 1.0));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state.optimize(getState().angle);
    driveMotor.set(state.speedMetersPerSecond / sdsModuleInterface.getPhysicalMaxSpeedMetersPerSecond());
    turningMotor.set(
        turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
  }

  public void setResetState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    driveMotor.set(state.speedMetersPerSecond / sdsModuleInterface.getPhysicalMaxSpeedMetersPerSecond());
    turningMotor.set(
        -(resetController.calculate(getAbsoluteEncoderAngle(), state.angle.getDegrees() + 180)));
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
    driveMotor.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition); 
  }

  public void setAbsoluteEncoderPosition(double radians) {
    absoluteEncoder.setPosition(Math.toDegrees(radians));
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
}
