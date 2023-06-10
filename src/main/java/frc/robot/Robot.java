// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sensors.UltrasonicAnalog;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Solenoid m_ultrasonic_solenoid =
      new Solenoid(
          Constants.CANDeviceIDs.kPCMID24V,
          PneumaticsModuleType.CTREPCM,
          Constants.UltrasonicConstants.kSensor_PCM_Channel);
  private Solenoid m_trainning_soloenoid;
  private DetectAprilTags detector = new DetectAprilTags();

  // UltrasonicAnalog sensor = new UltrasonicAnalog(Constants.GripperConstants.kSensorChannel);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();

    // turn on the solenoid channel to power the ultrasonic sensor
    m_ultrasonic_solenoid.set(true);

    // Starts up the camera and thread to detect april tags
    detector.initDetector();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Ultrasonic Range", UltrasonicAnalog.getInstance().getRange());
    DetectAprilTags.displayAprilTagInformation();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // ResetGrippers resetGrippers = new ResetGrippers();
    // resetGrippers.schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.getDriveTrain().resetEncoder();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // int brightness = (int) SmartDashboard.getNumber("Camera Brightness", 50);
    // m_robotContainer.setCameraBrightness(brightness);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    // enable the solenoid channel to allow for training the ultrasonic sensor
    m_trainning_soloenoid =
        new Solenoid(
            Constants.CANDeviceIDs.kPCMID24V,
            PneumaticsModuleType.CTREPCM,
            Constants.UltrasonicConstants.kTrainSensor_PCM_Channel);
    m_trainning_soloenoid.set(true);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
