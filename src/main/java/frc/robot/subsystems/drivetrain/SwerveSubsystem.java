package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft =
      new SwerveModule(
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftDriveEncoderReversed,
          DriveConstants.kBackLeftTurningEncoderReversed,
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight =
      new SwerveModule(
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightDriveEncoderReversed,
          DriveConstants.kBackRightTurningEncoderReversed,
          DriveConstants.kBackRightDriveAbsoluteEncoderPort,
          DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry odometer =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          new Rotation2d(0),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

  public SwerveSubsystem() {
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
              Constants.PITCH_OFFSET = getPitch();
            })
        .start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
    odometer.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);
  }

  @Override
  public void periodic() {
    odometer.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    // SmartDashboard.putNumber("LFDE", frontLeft.getDrivePosition());
    // SmartDashboard.putNumber("LBDE", backLeft.getDrivePosition());
    // SmartDashboard.putNumber("RFDE", frontRight.getDrivePosition());
    // SmartDashboard.putNumber("RBDE", backRight.getDrivePosition());
    // SmartDashboard.putNumber("LFSE", frontLeft.getTurningPosition());
    // SmartDashboard.putNumber("LBSE", backLeft.getTurningPosition());
    // SmartDashboard.putNumber("RFSE", frontRight.getTurningPosition());
    // SmartDashboard.putNumber("RBSE", backRight.getTurningPosition());
    // SmartDashboard.putNumber("LF RAD", frontLeft.getAbsoluteEncoderRad());
    // SmartDashboard.putNumber("LB RAD", backLeft.getAbsoluteEncoderRad());
    // SmartDashboard.putNumber("RF RAD", frontRight.getAbsoluteEncoderRad());
    // SmartDashboard.putNumber("RB RAD", backRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("LF DEG", frontLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("LB DEG", backLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("RF DEG", frontRight.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("RB DEG", backRight.getAbsoluteEncoderAngle());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  public double getAveragePosition() {
    double averagedistance =
        frontLeft.getDrivePosition()
            + frontRight.getDrivePosition()
            + backLeft.getDrivePosition()
            + backRight.getDrivePosition();
    return averagedistance / 4;
  }

  public double getPitch() {
    return gyro.getPitch();
  }
}
