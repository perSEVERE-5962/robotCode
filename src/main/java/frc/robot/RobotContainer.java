/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANDeviceIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ResetWheels;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeFeeder;
import frc.robot.commands.RunShooterFeeder;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.StopAll;
import frc.robot.sensors.Camera;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Notification;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.Feeder.*;
import frc.robot.subsystems.Intake.*;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.import
 * frc.robot.Constants.CANDeviceIDs;
 */
public class RobotContainer {
  private static RobotContainer instance;

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private final Notification notification = new Notification();
  private final Shooter shooter = new Shooter(CANDeviceIDs.kShooter1MotorID, CANDeviceIDs.kShooter2MotorID);



      
  // cameras
  private final Camera frontCamera;
  private final Camera backCamera;

  // Intake and feeder
  private final Intake intake = new Intake(true, CANDeviceIDs.kIntakeMotorID);
  private final Feeder feeder = new Feeder(true, CANDeviceIDs.kFeederMotorID);

  // Driver Controller
  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Trigger dr_resetToOffsets = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final Trigger dr_kLeftBumper= new JoystickButton( driverController, XboxController.Button.kLeftBumper.value);
  private final Trigger dr_kRightBumper= new JoystickButton( driverController, XboxController.Button.kRightBumper.value);
  private final Trigger dr_kButtonA = new JoystickButton(driverController, XboxController.Button.kA.value);

  // Test Controller
  private final XboxController testController = new XboxController(OIConstants.kCoPilotControllerPort);
  private final Trigger ts_kLeftBumper = new JoystickButton(testController, XboxController.Button.kLeftBumper.value);
  private final Trigger ts_lefttTrigger = new JoystickButton(testController, XboxController.Axis.kLeftTrigger.value);
  private final Trigger ts_kRightBumper = new JoystickButton(testController, XboxController.Button.kRightBumper.value);
  private final Trigger ts_rightTrigger = new JoystickButton(testController, XboxController.Axis.kRightTrigger.value);
  private final Trigger ts_buttonB = new JoystickButton(testController, XboxController.Button.kB.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    if (Constants.kUseJoystick) {
      driveTrain.setDefaultCommand(
        new DriveCommandWithThrottle(
            driveTrain,
            () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverRotAxis_Logitech),
            () -> driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx_Logitech),
            () -> driverController.getRawAxis(3)));
    } else {
      driveTrain.setDefaultCommand(
        new DriveCommand(
            driveTrain,
            () -> driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    }

    configureButtonBindings();
    frontCamera = new Camera(Constants.CameraConstants.kFrontCamera);
    backCamera = new Camera(Constants.CameraConstants.kBackCamera);
    SmartDashboard.putNumber("ShooterSpeed", 85);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    dr_resetToOffsets.onTrue(new ResetWheels(driveTrain));
    dr_kButtonA.onTrue(new TurnToAprilTag(1, 1));

    dr_kRightBumper.onTrue(new Shoot(shooter, feeder, notification));
    dr_kLeftBumper.onTrue(new IntakeNote(intake, notification, feeder));

    // ts_kRightBumper.onTrue(new SpinUpShooter(shooter));
    // ts_kLeftBumper.onTrue(new RunIntake(intake));
    // ts_rightTrigger.onTrue(new RunShooterFeeder(feeder));
    // ts_lefttTrigger.onTrue(new RunIntakeFeeder(feeder));
    // ts_buttonB.onTrue(new StopAll(feeder, intake, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command command = new Move(driveTrain, 0, 0, 0);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

    PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(DriveConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        DriveConstants.kPThetaController, 0, 0, DriveConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        driveTrain::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        driveTrain::setModuleStates,
        driveTrain);

    return new SequentialCommandGroup(
        new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> driveTrain.stopModules()));

  }

  public XboxController getDriverController() {
    return driverController;
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  public double getTargetShootVelocity() {
    return Constants.kmaxShooterRPM;
  }

}
