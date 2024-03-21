package frc.robot.subsystems.drivetrain;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem instance;

  public final SwerveModule frontLeft =
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
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0) * -1.0;
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

  String tab = "Autonomous";
  GenericEntry posXEntry = Shuffleboard.getTab(tab).add("Position X", 0.0).getEntry();
  GenericEntry posYEntry = Shuffleboard.getTab(tab).add("Position Y", 0.0).getEntry();
  GenericEntry rotEntry = Shuffleboard.getTab(tab).add("Rotation", 0.0).getEntry();
  GenericEntry teamColorEntry = Shuffleboard.getTab(tab)
    .add("Team color", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue", "blue", "colorWhenFalse", "red"))
    .getEntry();

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getHeading());
    SmartDashboard.putNumber("Average Distance Inches",getAverageDistanceInches() );
    odometer.update(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

    Pose2d pose = odometer.getPoseMeters();
    posXEntry.setDouble(pose.getX());
    posYEntry.setDouble(pose.getY());
    rotEntry.setDouble(pose.getRotation().getDegrees());
    teamColorEntry.setBoolean(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void move(double x, double y, double rot) {
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(x, y, rot);
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setResetStates(SwerveModuleState[] desiredStates) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(
    //    desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setResetState(desiredStates[0]);
    frontRight.setResetState(desiredStates[1]);
    backLeft.setResetState(desiredStates[2]);
    backRight.setResetState(desiredStates[3]);
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

  public void resetModuleEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
    // resetOdometry(getPose());
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public GenericEntry[] addDebugInfo() {
    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve Subsystem Debug");
    GenericEntry[] entries = {null, null, null, null};
    entries[0] = shuffleboardTab.add("BR Angle", 0).getEntry();
    entries[1] = shuffleboardTab.add("BL Angle", 0).getEntry();
    entries[2] = shuffleboardTab.add("FR Angle", 0).getEntry();
    entries[3] = shuffleboardTab.add("FL Angle", 0).getEntry();
    return entries;
    // shuffleboardTab.addNumber("Robot Heading", () -> getHeading());
    // shuffleboardTab.addString("Robot Location", () -> getPose().getTranslation().toString());
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
    setResetStates(
        new SwerveModuleState[] {
          // front left
          new SwerveModuleState(
              0.1,
              Rotation2d.fromDegrees(
                  Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg)),
          // front right
          new SwerveModuleState(
              0.1,
              Rotation2d.fromDegrees(
                  Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg)),
          // back left
          new SwerveModuleState(
              0.1,
              Rotation2d.fromDegrees(
                  Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg)),
          // back right
          new SwerveModuleState(
              0.1,
              Rotation2d.fromDegrees(
                  Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg))
        });
  }

  public boolean isAtAngle(double targetAngle) {
    boolean status = false;
    double currentAngle = Math.abs(getHeading());
    double absTargetAngle = Math.abs(targetAngle);

    if ((currentAngle >= (absTargetAngle + 1.0)) && (currentAngle <= (absTargetAngle - 1.0))) {
      status = true;
    }

    return status;
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
