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

  private GenericEntry backLeftEncoderAngle;
  private GenericEntry backRightEncoderAngle;
  private GenericEntry frontLeftEncoderAngle;
  private GenericEntry frontRightEncoderAngle;
  private GenericEntry backLeftEncoderAngleM360;
  private GenericEntry backRightEncoderAngleM360;
  private GenericEntry frontLeftEncoderAngleM360;
  private GenericEntry frontRightEncoderAngleM360;

  private SwerveSubsystem() {
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
            })
        .start();

    Constants.PITCH_OFFSET = getPitch();
    Constants.YAW_OFFSET = getYaw();

    String tab = Constants.tabs.kSwerveSubsystem;

    backLeftEncoderAngle = AddToShuffleboard.add(tab, "BL Angle", 0);
    backRightEncoderAngle = AddToShuffleboard.add(tab, "BR Angle", 0);
    frontLeftEncoderAngle = AddToShuffleboard.add(tab, "FL Angle", 0);
    frontRightEncoderAngle = AddToShuffleboard.add(tab, "FR Angle", 0);
    backLeftEncoderAngleM360 = AddToShuffleboard.add(tab, "BL Angle Mod 360", 0);
    backRightEncoderAngleM360 = AddToShuffleboard.add(tab, "BR Angle Mod 360", 0);
    frontLeftEncoderAngleM360 = AddToShuffleboard.add(tab, "FL Angle Mod 360", 0);
    frontRightEncoderAngleM360 = AddToShuffleboard.add(tab, "FR Angle Mod 360", 0);

    // frontLeftAngle = AddToShuffleboard.add("X Wheels", "Angle",
    // frontLeft.getAbsoluteEncoderRad());
    // AddToShuffleboard.add("X Wheels", "Target Angle", Math.toRadians(-45));
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

    double blAngle = backLeft.getAbsoluteEncoderAngle();
    double brAngle = backRight.getAbsoluteEncoderAngle();
    double flAngle = frontLeft.getAbsoluteEncoderAngle();
    double frAngle = frontRight.getAbsoluteEncoderAngle();
    backLeftEncoderAngle.setDouble(blAngle);
    backRightEncoderAngle.setDouble(brAngle);
    frontLeftEncoderAngle.setDouble(flAngle);
    frontRightEncoderAngle.setDouble(frAngle);
    backLeftEncoderAngleM360.setDouble(blAngle % 360);
    backRightEncoderAngleM360.setDouble(brAngle % 360);
    frontLeftEncoderAngleM360.setDouble(flAngle % 360);
    frontRightEncoderAngleM360.setDouble(frAngle % 360);

    // SmartDashboard.putNumber("Robot Heading", getHeading());
    // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    // frontLeftAngle.setDouble(frontLeft.getAbsoluteEncoderRad());
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

    public void setAbsloluteModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredAbsoluteState(desiredStates[0]);
    frontRight.setDesiredAbsoluteState(desiredStates[1]);
    backLeft.setDesiredAbsoluteState(desiredStates[2]);
    backRight.setDesiredAbsoluteState(desiredStates[3]);
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
        Math.abs(frontLeft.getDrivePosition())
            + Math.abs(frontRight.getDrivePosition())
            + Math.abs(backLeft.getDrivePosition())
            + Math.abs(backRight.getDrivePosition());
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
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(-45.0)),
          // front right
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45.0)),
          // back left
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(-135.0)),
          // back right
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135.0))
        });
  }

  public void setWheelsTo0() {
    setAbsloluteModuleStates(
        new SwerveModuleState[] {
          // front left
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg)),
          // front right
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg)),
          // back left
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg)),
          // back right
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg))
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