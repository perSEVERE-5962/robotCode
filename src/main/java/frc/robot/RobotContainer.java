/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.kauailabs.navx.frc.AHRS;
// import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team5962.camera.Camera;
import frc.robot.commands.*;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_copilotController = new XboxController(1);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_driveTrain;
  private Camera m_camera = new Camera();

  private SendableChooser<Integer> m_startPositionChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_startPositionChooser.setDefaultOption(
        "P1", Integer.valueOf(Constants.AutonomousStartPosition.position1));
    m_startPositionChooser.addOption(
        "P2", Integer.valueOf(Constants.AutonomousStartPosition.position2));

    SmartDashboard.putData("Auto Start Position", m_startPositionChooser);

    SmartDashboard.putNumber("Camera Brightness", 50);

    AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

    SwerveModule frontLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

    SwerveModule frontRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

    SwerveModule backLeftModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

    SwerveModule backRightModule =
        new MkSwerveModuleBuilder()
            .withLayout(
                shuffleboardTab
                    .getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L1)
            .withDriveMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

    m_driveTrain =
        new Drivetrain(m_gyro, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
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
    Command command;
    // int position = m_startPositionChooser.getSelected();
    // if (position == Constants.AutonomousStartPosition.position1) {
    // command = new AutoPos1(m_driveTrain);
    // } else if (position == Constants.AutonomousStartPosition.position2) {
    // command = new AutoPos2(m_driveTrain);
    // } else {
    // command = new StopDrive(m_driveTrain);
    // }
    command = new AutoDriveForward(0, m_driveTrain);
    return command;
  }

  public Command getTeleopCommand() {
    return new SwerveDriveCommand(m_driveTrain, m_driverController);
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
}
