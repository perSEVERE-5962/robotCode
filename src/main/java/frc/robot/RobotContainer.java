/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
  //private final Camera backCamera; // Intake/Note Detection

  // Intake and feeder
  private final Intake intake = Intake.getInstance();
  private final Feeder feeder = Feeder.getInstance();

  // Driver Controller
  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Trigger dr_resetToOffsets = new JoystickButton(driverController, XboxController.Button.kStart.value);
  private final Trigger dr_leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final Trigger dr_rightBumper = new JoystickButton(driverController,XboxController.Button.kRightBumper.value);
  //private final Trigger dr_buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final Trigger dr_buttonB = new JoystickButton(driverController, XboxController.Button.kB.value);

  // Test Controller
   private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);
   private final Trigger cp_leftBumper = new JoystickButton(copilotController, XboxController.Button.kLeftBumper.value);
   private final Trigger cp_rightBumper = new JoystickButton(copilotController, XboxController.Button.kRightBumper.value);

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
    //backCamera = new Camera(Constants.CameraConstants.kBackCamera);

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
    //dr_buttonA.onTrue(new TurnToAprilTag(1, 1));

    dr_rightBumper.onTrue(new Shoot(shooter, feeder, notification));
    dr_leftBumper.onTrue(new IntakeNote(intake, notification, feeder));
    dr_buttonB.onTrue(getAutonomousCommand());

    cp_leftBumper.toggleOnTrue(new OutIntake(intake));
    cp_rightBumper.toggleOnTrue(new OutShooterFeeder(feeder));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Command command = new Move(driveTrain, 0, 0, 0);
    Command command;
    if(SmartDashboard.getBoolean("redAutoPos1", true) || SmartDashboard.getBoolean("blueAutoPos1", true)){
        command = new AutoPosition1(driveTrain, shooter, feeder, notification, intake);
    }
    else if(SmartDashboard.getBoolean("redAutoPos3", true) || SmartDashboard.getBoolean("blueAutoPos3", true)){
      command = new AutoPosition3(driveTrain, shooter, feeder, notification, intake);
    }
    else{
      command = new AutoPosition2(driveTrain, shooter, feeder, notification, intake);
    }
    return command;
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
