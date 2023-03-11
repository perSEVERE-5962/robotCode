package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddToShuffleboard;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem instance;

  private final SwerveModule frontLeft =
      new SwerveModule(
          CANDeviceIDs.kFrontLeftDriveMotorID,
          CANDeviceIDs.kFrontLeftTurningMotorID,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          CANDeviceIDs.kFrontLeftDriveAbsoluteEncoderID,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight =
      new SwerveModule(
          CANDeviceIDs.kFrontRightDriveMotorID,
          CANDeviceIDs.kFrontRightTurningMotorID,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          CANDeviceIDs.kFrontRightDriveAbsoluteEncoderID,
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft =
      new SwerveModule(
          CANDeviceIDs.kBackLeftDriveMotorID,
          CANDeviceIDs.kBackLeftTurningMotorID,
          DriveConstants.kBackLeftDriveEncoderReversed,
          DriveConstants.kBackLeftTurningEncoderReversed,
          CANDeviceIDs.kBackLeftDriveAbsoluteEncoderID,
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight =
      new SwerveModule(
          CANDeviceIDs.kBackRightDriveMotorID,
          CANDeviceIDs.kBackRightTurningMotorID,
          DriveConstants.kBackRightDriveEncoderReversed,
          DriveConstants.kBackRightTurningEncoderReversed,
          CANDeviceIDs.kBackRightDriveAbsoluteEncoderID,
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

  private GenericEntry frontLeftPosEntry;
  private GenericEntry frontRightPosEntry;
  private GenericEntry backLeftPosEntry;
  private GenericEntry backRightPosEntry;
  private GenericEntry averagePosEntry;
  private GenericEntry averageDistEntry;

  private GenericEntry absolutePitchEntry;
  private GenericEntry relativePitchEntry;
  private GenericEntry absoluteYawEntry;
  private GenericEntry relativeYawEntry;

  private SwerveSubsystem() {
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
              Constants.PITCH_OFFSET = getPitch();
              Constants.YAW_OFFSET = getYaw();
            })
        .start();

    String tab = Constants.tabs.kSwerveSubsystem;

    frontLeftPosEntry = AddToShuffleboard.add(tab, "Front Left Pos", frontLeft.getDrivePosition());
    frontRightPosEntry =
        AddToShuffleboard.add(tab, "Front Right Pos", frontRight.getDrivePosition());
    backLeftPosEntry = AddToShuffleboard.add(tab, "Back Left Pos", backLeft.getDrivePosition());
    backRightPosEntry = AddToShuffleboard.add(tab, "Back Right Pos", backRight.getDrivePosition());
    averagePosEntry = AddToShuffleboard.add(tab, "Average Position", getAveragePosition());
    averageDistEntry = AddToShuffleboard.add(tab, "Average Distance", getAverageDistanceInches());

    tab = Constants.tabs.kAngle;

    absolutePitchEntry = AddToShuffleboard.add(tab, "Absolute Pitch", getPitch());
    relativePitchEntry =
        AddToShuffleboard.add(tab, "Relative Pitch", getPitch() - Constants.PITCH_OFFSET);
    absoluteYawEntry = AddToShuffleboard.add(tab, "Absolute Yaw", getYaw());
    relativeYawEntry = AddToShuffleboard.add(tab, "Relative Yaw", getYaw() - Constants.YAW_OFFSET);

    AddToShuffleboard.add(tab, "Pitch Offset", Constants.PITCH_OFFSET);
    AddToShuffleboard.add(tab, "Yaw Offset", Constants.YAW_OFFSET);
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

    frontLeftPosEntry.setDouble(frontLeft.getDrivePosition());
    frontRightPosEntry.setDouble(frontRight.getDrivePosition());
    backLeftPosEntry.setDouble(backLeft.getDrivePosition());
    backRightPosEntry.setDouble(backRight.getDrivePosition());
    averagePosEntry.setDouble(getAveragePosition());
    averageDistEntry.setDouble(getAverageDistanceInches());

    absolutePitchEntry.setDouble(getPitch());
    relativePitchEntry.setDouble(getPitch() - Constants.PITCH_OFFSET);
    absoluteYawEntry.setDouble(getYaw());
    relativeYawEntry.setDouble(getYaw() - Constants.PITCH_OFFSET);

    // SmartDashboard.putNumber("Robot Heading", getHeading());
    // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
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
    return averagedistance * 0.25;
  }

  public double convertPositionToDistance(double position) {
    return Units.metersToInches(position) / (Constants.ModuleConstants.kDriveEncoderRot2Inch);
  }

  public double convertDistanceToPosition(double distance) {
    return (distance * Constants.ModuleConstants.kDriveMotorGearRatio)
        / (Math.PI * Constants.ModuleConstants.kWheelDiameterInches);
  }

  public double getAverageDistanceInches() {
    return Math.abs(convertPositionToDistance(getAveragePosition()));
  }

  public void resetDrivePosition() {
    frontLeft.resetDriveEncoder();
    frontRight.resetDriveEncoder();
    backLeft.resetDriveEncoder();
    backRight.resetDriveEncoder();
    // resetOdometry(getPose());
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public void addDebugInfo() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve Subsystem Debug");
    shuffleboardTab.addNumber("Robot Heading", () -> getHeading());
    shuffleboardTab.addString("Robot Location", () -> getPose().getTranslation().toString());
    // shuffleboardTab.addNumber("LFDE", () -> frontLeft.getDrivePosition());
    // shuffleboardTab.addNumber("LBDE", () -> backLeft.getDrivePosition());
    // shuffleboardTab.addNumber("RFDE", () -> frontRight.getDrivePosition());
    // shuffleboardTab.addNumber("RBDE", () -> backRight.getDrivePosition());
    // shuffleboardTab.addNumber("LFSE", () -> frontLeft.getTurningPosition());
    // shuffleboardTab.addNumber("LBSE", () -> backLeft.getTurningPosition());
    // shuffleboardTab.addNumber("RFSE", () -> frontRight.getTurningPosition());
    // shuffleboardTab.addNumber("RBSE", () -> backRight.getTurningPosition());
    // shuffleboardTab.addNumber("LF RAD", () -> frontLeft.getAbsoluteEncoderRad());
    // shuffleboardTab.addNumber("LB RAD", () -> backLeft.getAbsoluteEncoderRad());
    // shuffleboardTab.addNumber("RF RAD", () -> frontRight.getAbsoluteEncoderRad());
    // shuffleboardTab.addNumber("RB RAD", () -> backRight.getAbsoluteEncoderRad());
    // shuffleboardTab.addNumber("LF DEG", () -> frontLeft.getAbsoluteEncoderAngle());
    // shuffleboardTab.addNumber("LB DEG", () -> backLeft.getAbsoluteEncoderAngle());
    // shuffleboardTab.addNumber("RF DEG", () -> frontRight.getAbsoluteEncoderAngle());
    // shuffleboardTab.addNumber("RB DEG", () -> backRight.getAbsoluteEncoderAngle());
  }

  /** Set the wheels to an X pattern to plant the robot. */
  public void setWheelsToX() {
    setModuleStates(
        new SwerveModuleState[] {
          // front left
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
          // front right
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
          // back left
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
          // back right
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
        });
  }

  /**
   * @return the instance
   */
  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }

    return instance;
  }
}
