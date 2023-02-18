/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final LinearSlide Position = null;
  private static final LinearSlide LinearSlide = null;
  private static final double Trajectory = 0;
  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick m_copilotController = new Joystick(OIConstants.kCoPilotControllerPort);

  private final Gripper m_gripper = new Gripper();
  private final Trigger m_buttonA = new JoystickButton(m_copilotController, 1);
  private final Trigger m_buttonB = new JoystickButton(m_copilotController, 2);
  // private AHRS m_Gyro = new AHRS(SPI.Port.kMXP);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_driveTrain = new SwerveSubsystem();
  private Camera m_camera = new Camera();
  // private LineDetector m_lineDetector = new LineDetector();
  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(
        new DriveCommand(
            m_driveTrain,
            () -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !m_driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    // Configure the button bindings
    configureButtonBindings();

    m_autonomousChooser.setDefaultOption(
        "Full Autonomous", new AUTO_LeaveCommunityAndEngage(m_driveTrain));
    m_autonomousChooser.addOption(
        "Past Community Line", new GroupSeqCom_MovePastLineWithoutColorSensor(m_driveTrain));
    m_autonomousChooser.addOption(
        "Onto Charging Station", new GroupParRace_GetOnChargingStation(m_driveTrain));

    SmartDashboard.putData("Auto Start Position", m_autonomousChooser);

    SmartDashboard.putNumber("Camera Brightness", 50);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, OIConstants.kZeroHeadingButtonIdx)
        .onTrue(new InstantCommand(() -> m_driveTrain.zeroHeading()));

    m_buttonA.onTrue(new ManipulatorClose(m_gripper));
    m_buttonB.onTrue(new ManipulatorOpen(m_gripper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Command command = m_autonomousChooser.getSelected();
    // command = new CrossLine();
    // command = new AutoDriveForward(0, m_driveTrain);
    return command;
    // return getAutonomousTrajectoryCommand();
  }

  private Command getAutonomousTrajectoryCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            m_driveTrain::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            m_driveTrain::setModuleStates,
            m_driveTrain);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_driveTrain.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_driveTrain.stopModules()));
  }

  public SwerveSubsystem getDriveTrain() {
    return m_driveTrain;
  }

  public void setCameraBrightness(int brightness) {
    m_camera.setBrightness(brightness);
  }

  public Camera getCamera() {
    return m_camera;
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
