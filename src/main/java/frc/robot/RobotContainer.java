/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.MotorCommutation;
// import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Gripper;
// import frc.robot.subsystems.LineDetector;
// import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private static final LinearSlide Position = null;
  // private static final LinearSlide LinearSlide = null;
  private final XboxController m_driverController = new XboxController(0);
  private final Joystick m_copilotController = new Joystick(1);

  private final Gripper m_gripper = new Gripper();
  private final Trigger m_buttonA = new JoystickButton(m_copilotController, 1);
  private final Trigger m_buttonB = new JoystickButton(m_copilotController, 2);
  // private AHRS m_Gyro = new AHRS(SPI.Port.kMXP);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_driveTrain = new Drivetrain();
  private Camera m_camera = new Camera();
  // private LineDetector m_lineDetector = new LineDetector();
  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveTrain.register();

    m_driveTrain.setDefaultCommand(
        new DriveCommand(
            m_driveTrain,
            () -> -modifyAxis(m_driverController.getLeftY()), // Axes are flipped here on purpose
            () -> -modifyAxis(m_driverController.getLeftX()),
            () -> -modifyAxis(m_driverController.getRightX())));

    // Configure the button bindings
    configureButtonBindings();

    m_autonomousChooser.setDefaultOption(
        "Full Autonomous", new AUTO_LeaveCommunityAndEngage(m_driveTrain));
    m_autonomousChooser.addOption(
        "Past Community Line", new GroupSeqCom_MovePastLineWithoutColorSensor(m_driveTrain));
    m_autonomousChooser.addOption(
        "Onto Charging Station", new GroupParRace_GetOnChargingStation(m_driveTrain));

    SmartDashboard.putData("Autonomous Mode", m_autonomousChooser);

    SmartDashboard.putNumber("Camera Brightness", 50);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
  }

  public Drivetrain getDriveTrain() {
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
