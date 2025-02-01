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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
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
      
  // Cameras
  //private final Camera frontCamera; 
  //private final Camera backCamera; 

  // Driver Controller
  private final XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Trigger dr_resetToOffsets = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kStart.value : 5);
   private final Trigger dr_leftBumper     = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kLeftBumper.value : 3);
  private final Trigger dr_rightBumper    = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kRightBumper.value : 4);
  private final Trigger dr_buttonA        = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kA.value : 12);
  private final Trigger dr_buttonB        = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kB.value : 11);
  private final Trigger dr_buttonX        = new JoystickButton(driverController, !Constants.kUseJoystick ? XboxController.Button.kX.value : 6);
 private final Trigger dr_buttonY = new JoystickButton(driverController, XboxController.Button.kY.value);
  // Copilot Controller
  // private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);
  // private final Trigger cp_leftBumper = new JoystickButton(copilotController, XboxController.Button.kLeftBumper.value);
  // private final Trigger cp_rightBumper = new JoystickButton(copilotController, XboxController.Button.kRightBumper.value);
  // private final Trigger cp_buttonB = new JoystickButton(copilotController, XboxController.Button.kB.value);
  // private final Trigger cp_buttonA = new JoystickButton(copilotController, XboxController.Button.kA.value);
  // private final Trigger cp_buttonX = new JoystickButton(copilotController, XboxController.Button.kX.value);
  // private final Trigger cp_buttonY = new JoystickButton(copilotController, XboxController.Button.kY.value);
  // private final Trigger cp_rightBumper = new JoystickButton(copilotController, XboxController.Button.kRightBumper.value);
  // private final Trigger cp_leftBumper = new JoystickButton(copilotController, XboxController.Button.kLeftBumper.value);

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

    m_autonomousChooser.setDefaultOption("No delay", getAutonomousCommand());
    m_autonomousChooser.addOption("Delayed 5 seconds", new SequentialCommandGroup(
      new Timer(5000),
      getAutonomousCommand()
    ));

    SmartDashboard.putData("Autonomous", m_autonomousChooser);
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
    dr_leftBumper.onTrue(new PickUpIntake());
    dr_buttonA.onTrue(new SetArmPosition(Constants.ScoringConstants.kL1));//trough
    dr_buttonB.onTrue(new SetArmPosition(Constants.ScoringConstants.kL2));//l2
    dr_buttonX.onTrue(new SetArmPosition(Constants.ScoringConstants.kL3));//l3
    dr_buttonY.onTrue(new SetArmPosition(Constants.ScoringConstants.kL4));//l4

    dr_rightBumper.whileTrue(new ShootWithIntake());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command;
    command= new ExampleAuto();
    return command;
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }
  public SwerveSubsystem getDrivetrainSubsystem() {
    return driveTrain;
   }

}
