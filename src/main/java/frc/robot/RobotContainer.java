/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Notification.NoteState;
import frc.robot.subsystems.drivetrain.*;
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
  private final Notification notification = Notification.getInstance();
  private final Shooter shooter = Shooter.getInstance();
      
  // Cameras
  //private final Camera frontCamera; // shooter/april tag
  @SuppressWarnings(value = "unused")
  private final Camera backCamera; // Intake/Note Detection

  // Intake and feeder
  private final Intake intake = Intake.getInstance();
  private final Feeder feeder = Feeder.getInstance();

  // Driver Controller
  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Trigger dr_resetToOffsets = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final Trigger dr_leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final Trigger dr_rightBumper = new JoystickButton(driverController,XboxController.Button.kRightBumper.value);
  private final Trigger dr_buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final Trigger dr_buttonB = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final Trigger dr_buttonX = new JoystickButton(driverController, XboxController.Button.kX.value);
  // Test Controller
   private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);
  //  private final Trigger cp_leftBumper = new JoystickButton(copilotController, XboxController.Button.kLeftBumper.value);
  //  private final Trigger cp_rightBumper = new JoystickButton(copilotController, XboxController.Button.kRightBumper.value);
   private final Trigger cp_buttonB = new JoystickButton(copilotController, XboxController.Button.kB.value);
   private final Trigger cp_buttonA = new JoystickButton(copilotController, XboxController.Button.kA.value);
   //private final Trigger cp_buttonX = new JoystickButton(copilotController, XboxController.Button.kX.value);
   //private final Trigger cp_buttonY = new JoystickButton(copilotController, XboxController.Button.kY.value);

  // Autonomous
  private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

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

    //frontCamera = new Camera(Constants.CameraConstants.kFrontCamera);
    backCamera = new Camera(Constants.CameraConstants.kBackCamera);

    m_autonomousChooser.setDefaultOption("Default", new Move(driveTrain, 0, 0, 0));
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
    dr_rightBumper.onTrue(new Shoot());
    dr_leftBumper.onTrue(new IntakeNote());
    dr_buttonA.onTrue(new ShootWithApriltag());
    dr_buttonB.onTrue(new FullAutonomousMiddleNoteShooting());
    dr_buttonX.onTrue(new AutonomousShootMiddleNote());

    // cp_leftBumper.toggleOnTrue(new OutIntake(intake));
    // cp_rightBumper.toggleOnTrue(new OutShooterFeeder(feeder));
    cp_buttonB.onTrue(new StopAll(feeder, intake, shooter));
    cp_buttonA.onTrue(new ResetNoteStatus());

    //cp_buttonY.onTrue(new Move(driveTrain, 0, 0, -1));
    //cp_buttonX.onTrue(new Move(driveTrain, 0, 0, 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command;
    NetworkTableInstance networktable=NetworkTableInstance.getDefault();
    NetworkTable table = networktable.getTable("AutomonusSelect");
    double autoPosition = table.getEntry("Close Note").getDouble(1);
    if(autoPosition == 1) {
      command = new FullAutonomousMiddleNoteShooting();
    } else {
      command = new AutoPosition2();
    }

    driveTrain.resetOdometry(driveTrain.getPose());
    return command;
  }

  // public XboxController getDriverController() {
  //   return driverController;
  // }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  // public double getTargetShootVelocity() {
  //   return Constants.kmaxShooterRPM;
  // }

  public void resetNoteState() {
    notification.updateState(NoteState.NOTE_NOT_IN_POSSESSION);
  }
}
