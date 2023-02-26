/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.manipulator.*;
import frc.robot.sensors.ColorSensor;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.subsystems.manipulator.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_copilotController =
      new XboxController(OIConstants.kCoPilotControllerPort);
  Trigger co_yButton = new JoystickButton(m_copilotController, XboxController.Button.kY.value);
  Trigger co_aButton = new JoystickButton(m_copilotController, XboxController.Button.kA.value);
  Trigger co_bButton = new JoystickButton(m_copilotController, XboxController.Button.kB.value);
  Trigger co_xButton = new JoystickButton(m_copilotController, XboxController.Button.kX.value);
  Trigger dr_aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger dr_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  Trigger dr_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger dr_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger co_lBumper =
      new JoystickButton(m_copilotController, XboxController.Button.kLeftBumper.value);
  Trigger co_rBumper =
      new JoystickButton(m_copilotController, XboxController.Button.kRightBumper.value);
  Trigger dr_rBumper =
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  Trigger dr_lBumper =
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_driveTrain = SwerveSubsystem.getInstance();
  // private Camera m_camera = new Camera();
  private final ColorSensor m_colorSensor = new ColorSensor();

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
    configureShuffleBoard();

    m_autonomousChooser.setDefaultOption(
        "Full Autonomous", new AUTO_LeaveCommunityAndEngage(m_driveTrain));
    m_autonomousChooser.addOption(
        "Cross Line over Charge Station",
        new GroupSeqCom_MovePastLineWithoutColorSensor(m_driveTrain));
    m_autonomousChooser.addOption("Cross Line", new GroupSeqCom_MovePastLine(m_driveTrain));

    SmartDashboard.putData("Autonomous Mode", m_autonomousChooser);

    // SmartDashboard.putNumber("Camera Brightness", 50);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("CameraPublisher");
    NetworkTable subtable = datatable.getSubTable("MainCamera");
    NetworkTableEntry streamsEntry = subtable.getEntry("streams");
    streamsEntry.setString("mjpeg:http://10.59.62.52:1181/stream.mjpg");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    co_aButton.onTrue(new ScorePostion1());
    co_bButton.onTrue(new ScorePostion2());
    co_yButton.onTrue(new ScorePostion3());
    co_xButton.onTrue(new ResetPosition());
    dr_aButton.onTrue(new ScoreCubePosition1());
    dr_bButton.onTrue(new ScoreCubePosition2());
    dr_yButton.onTrue(new ScoreCubePosition3());
    dr_xButton.onTrue(new ResetCubePosition());
    co_lBumper.onTrue(new AlignGripperToDoubleSubstation());
    co_rBumper.onTrue(new GrabCone());
    dr_lBumper.onTrue(new AlignGripperToDoubleSubstation());
    dr_rBumper.onTrue(new GrabCube());

    /*     new JoystickButton(m_driverController, OIConstants.kZeroHeadingButtonIdx)
        .onTrue(new InstantCommand(() -> m_driveTrain.zeroHeading()));

    m_grabCone.onTrue(new GripperClose());
    m_releaseCone.onTrue(new GripperOpen());

    m_gridPos1.onTrue(new ScoreConeOnGridPos1());
    m_gridPos2.onTrue(new ScoreConeOnGridPos2());
    m_gridPos3.onTrue(new ScoreConeOnGridPos3());

    m_initSubstation.onTrue(new AlignGripperToDoubleSubstation());
    m_retrieveFromSubstation.onTrue(new GetConeFromDoubleSubstation());
    m_resetManipulator.onTrue(new ResetManipulator()); */
  }

  private void configureShuffleBoard() {
    ShuffleboardTab shuffleboardTab;
    // Angle tab
    shuffleboardTab = Shuffleboard.getTab("Angle");
    shuffleboardTab.addNumber("Pitch Offset", () -> Constants.PITCH_OFFSET);
    shuffleboardTab.addNumber("Pitch", () -> getDriveTrain().getPitch());
    shuffleboardTab.addNumber(
        "Relative Pitch", () -> getDriveTrain().getPitch() - Constants.PITCH_OFFSET);

    // Color sensor
    shuffleboardTab = Shuffleboard.getTab("Line Detector");
    shuffleboardTab.addNumber("Confidence", () -> m_colorSensor.getConfidence());
    shuffleboardTab.addString("Color Name", () -> m_colorSensor.getColor());
    shuffleboardTab.addString("Hex Value", () -> m_colorSensor.getHex().toString());

    // Wheels
    shuffleboardTab = Shuffleboard.getTab("Wheels");
    shuffleboardTab.addNumber("Average Position", () -> getDriveTrain().getAveragePosition());

    // Manipulator
    shuffleboardTab = Shuffleboard.getTab("Manipulators");
    shuffleboardTab.addNumber("Lift Encoder Position", () -> Lift.getInstance().getPosition());
    shuffleboardTab.addNumber("Reach Encoder Position", () -> Reach.getInstance().getPosition());
    shuffleboardTab.addNumber("Wrist Encoder Position", () -> Wrist.getInstance().getPosition());

    // Other
    getDriveTrain().addDebugInfo();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = m_autonomousChooser.getSelected();
    return command;
  }

  // private Command getAutonomousTrajectoryCommand() {
  //   // 1. Create trajectory settings
  //   TrajectoryConfig trajectoryConfig =
  //       new TrajectoryConfig(
  //               AutoConstants.kMaxSpeedMetersPerSecond,
  //               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //           .setKinematics(DriveConstants.kDriveKinematics);

  //   // 2. Generate trajectory
  //   Trajectory trajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
  //           new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
  //           trajectoryConfig);

  //   // 3. Define PID controllers for tracking trajectory
  //   PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  //   PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  //   ProfiledPIDController thetaController =
  //       new ProfiledPIDController(
  //           AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   // 4. Construct command to follow trajectory
  //   SwerveControllerCommand swerveControllerCommand =
  //       new SwerveControllerCommand(
  //           trajectory,
  //           m_driveTrain::getPose,
  //           DriveConstants.kDriveKinematics,
  //           xController,
  //           yController,
  //           thetaController,
  //           m_driveTrain::setModuleStates,
  //           m_driveTrain);

  //   // 5. Add some init and wrap-up, and return everything
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> m_driveTrain.resetOdometry(trajectory.getInitialPose())),
  //       swerveControllerCommand,
  //       new InstantCommand(() -> m_driveTrain.stopModules()));
  // }

  private SwerveSubsystem getDriveTrain() {
    return m_driveTrain;
  }

  // public void setCameraBrightness(int brightness) {
  //   m_camera.setBrightness(brightness);
  // }

  // public Camera getCamera() {
  //   return m_camera;
  // }
}
