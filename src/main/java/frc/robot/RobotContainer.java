/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.CameraLight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.sensors.PIDControl;

import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick driverController = new Joystick(0);
  private final Joystick copilotController = new Joystick(1);
  // private final JoystickButton joyButton = new
  // JoystickButton(copilotController, 5);
  // private final JoystickButton joyButton6 = new
  // JoystickButton(copilotController, 6);

  // arm buttons
  private final JoystickButton buttonA = new JoystickButton(copilotController, 1);
  private final JoystickButton buttonB = new JoystickButton(copilotController, 2);
  private final JoystickButton button8 = new JoystickButton(copilotController, 8);
  private final JoystickButton button7 = new JoystickButton(copilotController, 7);

  // camera led buttons
  private final JoystickButton buttonY = new JoystickButton(copilotController, 3);
  private final JoystickButton buttonX = new JoystickButton(copilotController, 4);

  //servo buttons
  private final JoystickButton buttonRB = new JoystickButton(copilotController, 6);
  private final JoystickButton buttonLB = new JoystickButton(copilotController, 5);

  private SendableChooser driveChooser = new SendableChooser<Command>();
  private SendableChooser autoChooser = new SendableChooser<Command>();

  // The robot's subsystems and commands are defined here...
  private final Drive driveSubsystem = new Drive();
  private final Intake intake = new Intake();
  private final Winch winchSubsystem = new Winch();
  private final AutoCommand autoCommand = new AutoCommand(driveSubsystem);
  private final CameraLight cameraLight = new CameraLight();
  private final Elevator elevatorsubsystem = new Elevator();
  private final ArmServo servo = new ArmServo();
  private final Arm arm = new Arm();
  private final ShakeArm shakeArm = new ShakeArm(arm);


  public Command getShakeArm(){
    return shakeArm;
  }

  // private final RunTankDrive driveCommand = new RunTankDrive(driveSubsystem);

  // private final WinchUp winchUp = new WinchUp(winchSubsystem);


  private final PIDControl pidControl = new PIDControl();
  private final PathFollow followPath = new PathFollow(driveSubsystem, pidControl, driveSubsystem.getGyro());

  // private final ColorSensor colorSensor = new ColorSensor();
  // private final ControlPanel controlPanel = new ControlPanel(colorSensor);
  // private final SenseColor senseColorCommand = new SenseColor(colorSensor);
  // private final SpinToColor spinColorCommand = new SpinToColor(controlPanel);
  // private final SpinRotations spinRotCommand = new SpinRotations(controlPanel);
  private final InchForward inchForward = new InchForward(driveSubsystem);
  private Command driveCommand;
  private Command myautoCommand;
  private final RunIntake runIntake = new RunIntake(intake);
  private final Shoot shoot = new Shoot(intake);
  private final TurnOnLight lightOn = new TurnOnLight(cameraLight);
  private final TurnOffLight lightOff = new TurnOffLight(cameraLight);
  private final MoveArmVision armVision = new MoveArmVision(arm);
  private final ElevatorUp elevatorUp = new ElevatorUp(elevatorsubsystem);
  private final ElevatorDown elevatorDown = new ElevatorDown(elevatorsubsystem);
  private final GetCamera cameraCommand = new GetCamera();
  // private final CPSubsystem cpSubsystem = new CPSubsystem();
  private AutoSequence autoSequence = new AutoSequence(arm, cameraLight, driveSubsystem, intake, pidControl);
  private ThreeBallAuto threeBallAuto = new ThreeBallAuto(arm, cameraLight, driveSubsystem, intake);
  private ThreeBallAutoRight threeBallAutoRight = new ThreeBallAutoRight(arm, cameraLight, driveSubsystem, intake);

  private DriveLeft left = new DriveLeft(driveSubsystem);
  private DriveRight right = new DriveRight(driveSubsystem);
  private StopDrive stop = new StopDrive(driveSubsystem);
  private DriveForward goForward = new DriveForward(driveSubsystem);
  private DriveBackwards goBackwards = new DriveBackwards(driveSubsystem);
  private StopArm stopArm = new StopArm();
  // private WinchUp winchUp = new WinchUp();
  private final double shootAngle = 13.0;


  public Arm getArm() {
    return arm;
  }
  public Command getAutoSequence(){
    return autoSequence;
  }
  public double getIntake() {
    double axisValue = copilotController.getRawAxis(1);
    return axisValue;
  }

  public double getElevatorUpAxis() {
    double axisValue = copilotController.getRawAxis(3);
    return axisValue;
  }

  public double getElevatorDownAxis() {
    double axisValue = copilotController.getRawAxis(2);
    return axisValue;
  }

  public Command getTurnOnLight() {
    return lightOn;
  }

  public Command getTurnOffLight() {
    return lightOff;
  }

  public Command getArmVision() {
    return armVision;
  }

  public Command getInchForward() {
    return inchForward;
  }

  public Command getRunIntake() {
    return runIntake;
  }

  public Command getCamera() {
    return cameraCommand;

  }

  public Command getShoot() {
    return shoot;
  }

  public Command getElevatorUp() {
    return elevatorUp;
  }

  public Command getElevatorDown() {
    return elevatorDown;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveChooser.setDefaultOption("JamieDrive", new RunJamieDrive(driveSubsystem));
    //driveChooser.addOption("smooth tankdrive", new SmoothTankDrive(driveSubsystem));
    //driveChooser.addOption("smooth arcadedrive", new SmoothArcadeDrive(driveSubsystem));
    //driveChooser.addOption("tankdrive", new RunTankDrive(driveSubsystem));
    //driveChooser.addOption("arcadedrive", new ArcadeDrive(driveSubsystem));
    SmartDashboard.putData("drivercontrol", driveChooser);
    SmartDashboard.putBoolean("Use PathFollower", true);
    autoChooser.setDefaultOption("Three Ball Auto (Left)", threeBallAutoRight);
     autoChooser.addOption("Five Ball Auto", autoSequence);
     autoChooser.addOption("Three Ball Auto (Right)", threeBallAuto);

     SmartDashboard.putData("auto chooser", autoChooser);
  }
  
  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonA.whenPressed(new MoveArmToIntake(arm));
    buttonB.whenPressed(new MoveArmToShoot(arm, shootAngle));
    buttonX.whenPressed(new TurnOnLight(cameraLight));
    buttonY.whenPressed(new TurnOffLight(cameraLight));
    buttonRB.whenPressed(new MoveServoToOpen(servo));
    buttonLB.whenPressed(new MoveServoToClose(servo));
    // button8.whenPressed(new ResetArm());
    button7.whileHeld(new WinchUp(winchSubsystem));
  }

  public void moveArmToShoot() {
    Command move = new MoveArmToShoot(arm,shootAngle);
    if (move != null) {
      move.schedule();
    }
  }

  public boolean armInShoootPosition() {
    return arm.isInShootPosition(shootAngle);
  }

  public double getArmPosition() {
    return arm.getEncoderValues();
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }

  public Command getDriveCommand() {
    driveCommand = (Command) driveChooser.getSelected();
    return driveCommand;
  }
  public Command getAutoCommand() {
    myautoCommand = (Command) autoChooser.getSelected();
    return myautoCommand;
  }

  public Command getTurnLeftCommand() {
    return left;
  }

  public DriveRight getTurnRightCommand() {
    return right;
  }

  public Joystick getDriverJoystick() {
    return driverController;
  }

  public Joystick getCopilotJoystick() {
    return copilotController;
  }

  public Command getFollowPath() {
    return followPath;
  }

  public Command getgoForward() {
    return goForward;
  }

  public Command getgoBackwards() {
    return goBackwards;
  }

  public Command getStopArm() {
    return stopArm;
  }

  // public Command getWinchUp() {
  // return winchUp;
  // }

  // public Command getSenseColorCommand() {
  // return senseColorCommand;
  // }

  // public Command getSpinColorCommand() {
  // return spinColorCommand;
  // }

  // public Command getSpinRotCommand() {
  // return spinRotCommand;
  // }

  public Command stopdrive() {
    return stop;
  }

  public Command driveLeft() {
    return null;
  }

  public Command driveRight() {
    return null;
  }

  public Drive getDrive() {
    return driveSubsystem;
  }



  private String getCurrentTime() {
    SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd@HH-mm-ss", Locale.US);
    return dateFormat.format(new Date());
  }

  public void putMessage(String message) {
    String logString = getCurrentTime() + " " + message + System.lineSeparator();
    System.out.println(logString);
  }

  public boolean isUsingPathFollower() {
    return SmartDashboard.getBoolean("Use PathFollower", true);
  }
}