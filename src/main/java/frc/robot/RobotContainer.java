/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Joystick m_copilotController = new Joystick(1);
  private Intake m_intake = new Intake();
  private IntakeSpeed m_intakeSpeed = new IntakeSpeed(m_intake, m_copilotController);

  private Arm m_arm = new Arm();
  private moveArm m_moveArm = new moveArm(m_copilotController, m_arm);
  private final Joystick m_driverController = new Joystick(0);
  private AutoPickupBall m_autoPickupBall = new AutoPickupBall(m_intake, m_driveTrain, m_arm, m_gyro);
  private AutoDriveScore m_autoShootBall = new AutoDriveScore(m_driveTrain, m_intake);
  private GyroTesting m_gyroTesting = new GyroTesting(m_driveTrain, m_intake, m_arm, m_gyro);

  private SendableChooser<Command> m_driveChooser = new SendableChooser<>();
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private SendableChooser<Integer> m_motorControllerChooser = new SendableChooser<>();

  private Camera m_camera = new Camera();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_motorControllerChooser.setDefaultOption(
        "Spark Max", Integer.valueOf(Constants.MotorControllerType.kREV));
    m_motorControllerChooser.addOption(
        "Talon SRX/Victor SPX", Integer.valueOf(Constants.MotorControllerType.kCTRE));
    SmartDashboard.putData("drivetrain motor controller", m_motorControllerChooser);

    m_driveChooser.setDefaultOption(
        "tank drive", new RunTankDrive(m_driveTrain, m_driverController));
    m_driveChooser.addOption(
        "Two Stick Arcase", new TwoStickArcade(m_driveTrain, m_driverController));
    m_driveChooser.addOption(
        "One Stick Arcase", new OneStickArcade(m_driveTrain, m_driverController));
    SmartDashboard.putData("driver control", m_driveChooser);

    m_autoChooser.setDefaultOption("pickup ball", m_autoPickupBall);
    m_autoChooser.addOption("shoot ball", m_autoShootBall);
    m_autoChooser.addOption("test gyro", m_gyroTesting);
    SmartDashboard.putData("auto chooser", m_autoChooser);

    SmartDashboard.putNumber("camera brightness", 50);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (Command) m_autoChooser.getSelected();
  }

  public Command getDriveCommand() {
    return (Command) m_driveChooser.getSelected();
  }

  public void setMotorControllerType() {
    m_driveTrain.setMotorControllerType(
        ((Integer) m_motorControllerChooser.getSelected()).intValue());
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  public IntakeSpeed getIntakeSpeed() {
    return m_intakeSpeed;
  }

  public Command getArm() {
    return m_moveArm;
  }

  public void setCameraBrightness(int brightness) {
    m_camera.setBrightness(brightness);
  }

  public AHRS getGyro() {
    return m_gyro;
  }
}
