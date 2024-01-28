/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.*;
import frc.robot.Constants.CANDeviceIDs;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance;
  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_driveTrain = SwerveSubsystem.getInstance();
  private final Intake intake=new Intake(false,CANDeviceIDs.kIntakeMotorID);
  private final Intake feeder=new Intake(true,CANDeviceIDs.kFeederMotorID);

  Trigger dr_resetToOffsets =
      new JoystickButton(m_driverController, XboxController.Button.kStart.value);
      Trigger dr_aButton = new JoystickButton(m_driverController ,XboxController.Button.kA.value);
      Trigger dr_bButton = new JoystickButton(m_driverController ,XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private RobotContainer() {
    /*m_driveTrain.setDefaultCommand(
        new DriveCommand(
            m_driveTrain,
            () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis_Logitech),
            () -> m_driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx_Logitech)));*/
  

    m_driveTrain.setDefaultCommand(
        new DriveCommandWithThrottle(
            m_driveTrain,
            () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis_Logitech),
            () -> m_driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx_Logitech),
            () -> m_driverController.getRawAxis(3)));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    dr_resetToOffsets.onTrue(new ResetWheels(m_driveTrain));
    dr_aButton.toggleOnTrue(new RunIntake(intake));
    dr_bButton.toggleOnTrue(new RunFeeder(feeder));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = new Move(m_driveTrain, 0, 0, 0);
    return command;
  }

  public XboxController getDriverController() {
    return m_driverController;
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }
}
