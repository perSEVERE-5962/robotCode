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
  private final Trigger dr_ResestAllParts = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final Trigger dr_ButtonB = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final Trigger dr_ButtonX = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final Trigger dr_ButtonA = new JoystickButton(driverController, XboxController.Button.kA.value);

  // Copilot Controller
  private final XboxController copilotController = new XboxController(OIConstants.kCoPilotControllerPort);
  private final Trigger cp_ReefLevel1Right = new JoystickButton(copilotController, XboxController.Button.kLeftBumper.value);
  private final Trigger cp_ReefLevel2Right = new JoystickButton(copilotController, XboxController.Button.kRightBumper.value);
  private final Trigger cp_ReefLevel3Right = new JoystickButton(copilotController, XboxController.Button.kLeftStick.value);
  private final Trigger cp_ReefLevel4Right = new JoystickButton(copilotController, XboxController.Button.kRightStick.value);
  private final Trigger cp_ReefLevel2Left = new JoystickButton(copilotController, XboxController.Button.kB.value);
  private final Trigger cp_ReefLevel3Left = new JoystickButton(copilotController, XboxController.Axis.kLeftTrigger.value);
  private final Trigger cp_ReefLevel4Left = new JoystickButton(copilotController, XboxController.Axis.kRightTrigger.value);
  private final Trigger cp_CoralStation = new JoystickButton(copilotController, XboxController.Button.kBack.value);
  private final Trigger cp_CollectCoral = new JoystickButton(copilotController, XboxController.Button.kStart.value);
  private final Trigger cp_ScoreCoral = new JoystickButton(copilotController, XboxController.Button.kY.value);

  //testing controller
  private final XboxController testingController = new XboxController(OIConstants.kTestingControllerPort);
  private final Trigger tc_ForwardPivot = new JoystickButton(testingController, XboxController.Button.kRightBumper.value);
  private final Trigger tc_BackwardPivot = new JoystickButton(testingController, XboxController.Button.kLeftBumper.value);
  //private final Trigger tc_ForwardWrist = new JoystickButton(testingController, XboxController.Button.kA.value);
  private final Trigger tc_BackwardWrist = new JoystickButton(testingController, XboxController.Button.kB.value);
  private final Trigger tc_ForwardReach = new JoystickButton(testingController, XboxController.Button.kX.value);
  private final Trigger tc_BackwardReach = new JoystickButton(testingController, XboxController.Button.kY.value);
  private final Trigger tc_TestPivot = new JoystickButton(testingController, XboxController.Button.kA.value);
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
/* 
    m_autonomousChooser.setDefaultOption("Center", getAutonomousCommand(1));
    m_autonomousChooser.addOption("Right", getAutonomousCommand(0));
    m_autonomousChooser.addOption("left", getAutonomousCommand(2));

    SmartDashboard.putData("Autonomous", m_autonomousChooser); */
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
    //dr_ResestAllParts.onTrue(new  ResetFuuctionComplete());
  //dr_ButtonB.onTrue(new ResetArmAndWrist(0));
    cp_ReefLevel1Right.onTrue(new MoveAndScoreCoral(Constants.ScoringConstants.kL1, true));//trough
    //cp_ReefLevel2.onTrue(new SetArmPosition(Constants.ScoringConstants.kL2));//l2
    cp_ReefLevel2Right.onTrue(new MoveAndScoreCoral(Constants.ScoringConstants.kL2, true));//l2
    cp_ReefLevel3Right.onTrue(new MoveAndScoreCoral(Constants.ScoringConstants.kL3, true));
    cp_ReefLevel2Left.onTrue(new MoveAndScoreCoral(Constants.ScoringConstants.kL2, false));//l2
    cp_ReefLevel3Left.onTrue(new MoveAndScoreCoral(Constants.ScoringConstants.kL3, false));
    //cp_ReefLevel3Right.whileTrue(new moveSubsystems(-0.7, "pivotSub")); // move pivot back - towards starting point
    //cp_ReefLevel4.onTrue(new SetArmPosition(Constants.ScoringConstants.kL4));//l4    
    //cp_ReefLevel4Right.whileTrue(new moveSubsystems(0.7, "pivotSub")); // move pivot forward - towards scoring position
    cp_CoralStation.onTrue(new SetReachPosition(6).andThen(new SetArmPosition(Constants.ScoringConstants.kStation)));//Coral Station

    cp_CollectCoral.onTrue(new  CollectCoral());
    cp_ScoreCoral.whileTrue(new ScoreCoral());//-5

    dr_ButtonB.whileTrue(new ResetFunctionComplete());
    dr_ButtonA.whileTrue(new MoveToReefWithTags(true));
    

    //tc_ForwardWrist.onTrue(new SetPivotPosition(8));
    tc_ForwardPivot.whileTrue(new moveSubsystems(1.0, "pivotSub"));
    tc_BackwardPivot.whileTrue(new moveSubsystems(-1.0, "pivotSub"));
    //tc_ForwardWrist.whileTrue(new moveSubsystems(0.4, "wristSub"));
    tc_BackwardWrist.whileTrue(new moveSubsystems(-0.4, "wristSub"));
    tc_ForwardReach.whileTrue(new moveSubsystems(0.4, "reachSub"));
    tc_BackwardReach.whileTrue(new moveSubsystems(-0.4, "reachSub"));
    tc_TestPivot.onTrue(new ResetFunctionComplete());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command;
  
    command=new SetArmPosition(1)./*andThen( new ResetWheels(driveTrain).*/andThen(new AutoToTroughWithCamera());
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
