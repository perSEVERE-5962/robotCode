/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Notification;
import frc.robot.sensors.UltrasonicAnalog;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_driveTrain = SwerveSubsystem.getInstance();
  private final Notification m_notification = new Notification();
  private final Shooter shooter = new Shooter(CANDeviceIDs.kShooter1MotorID, CANDeviceIDs.kShooter2MotorID);
  private final Intake intake = new Intake(true, CANDeviceIDs.kIntakeMotorID);
  private final Intake feeder = new Intake(false, CANDeviceIDs.kFeederMotorID);
  private final UltrasonicAnalog feederUltrasonic = new UltrasonicAnalog(UltrasonicConstants.kFeeder_Analog_Channel,
      UltrasonicConstants.kFeeder_PCM_Channel);
  private final UltrasonicAnalog intakeUltrasonic = new UltrasonicAnalog(UltrasonicConstants.kIntake_Analog_Channel,
      UltrasonicConstants.kIntake_PCM_Channel);

  Trigger dr_resetToOffsets = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  Trigger dr_aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  Trigger dr_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  //Trigger dr_runTheShooter = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  Trigger dr_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  Trigger dr_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private Solenoid m_intake_solenoid = new Solenoid(
      Constants.CANDeviceIDs.kPCMID24V,
      PneumaticsModuleType.REVPH,
      Constants.UltrasonicConstants.kIntake_PCM_Channel);
  private Solenoid m_feeder_solenoid = new Solenoid(
      Constants.CANDeviceIDs.kPCMID24V,
      PneumaticsModuleType.REVPH,
      Constants.UltrasonicConstants.kFeeder_PCM_Channel);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    m_driveTrain.setDefaultCommand(
        new DriveCommand(
            m_driveTrain,
            () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
            () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
            () -> m_driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    /*
     * m_driveTrain.setDefaultCommand(
     * new DriveCommandWithThrottle(
     * m_driveTrain,
     * () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
     * () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
     * () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis_Logitech),
     * () -> m_driverController.getRawButton(OIConstants.
     * kDriverFieldOrientedButtonIdx_Logitech),
     * () -> m_driverController.getRawAxis(3)));
     */

    configureButtonBindings();
    m_intake_solenoid.set(true);
    m_feeder_solenoid.set(true);

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
    dr_resetToOffsets.onTrue(new ResetWheels(m_driveTrain));

    dr_bButton.toggleOnTrue(new IntakeNote(intake, intakeUltrasonic, feederUltrasonic, m_notification, feeder));   
    //dr_xButton.onTrue(new TurnToZero(m_driveTrain, 1));
    dr_xButton.onTrue(new MoveWithDistance(m_driveTrain, -0.5, 33));      
    dr_aButton.toggleOnTrue(new RunShooterFeeder(feeder,feederUltrasonic));
    dr_yButton.toggleOnTrue(new RunIntake(intake, intakeUltrasonic));
   // dr_runTheShooter.onTrue(new Shoot(shooter, feeder, feederUltrasonic, m_notification)); 
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
  public double getTargetShootVelocity(){
    return Constants.kmaxShooterRPM;
  }
}

