// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DetectAprilTags;

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
        Constants.UltrasonicConstants.kIntake_PCM_Channel);

  private int speakerTag1Id = 0;
  private int speakerTag2Id = 0;
  private TagInfo speakerTag1Info;
  private TagInfo speakerTag2Info;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_ultrasonic_solenoid.set(true);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();
    DetectAprilTags.activate();
    // Speaker IDs
    // speakerTag1Id = SpeakerTagInfo.kTeamColor == SpeakerTagInfo.TEAM_COLOR_BLUE ? SpeakerIds.kBlueSpeakerAprilTag1Id : SpeakerIds.kRedSpeakerAprilTag1Id;
    // speakerTag2Id = SpeakerTagInfo.kTeamColor == SpeakerTagInfo.TEAM_COLOR_BLUE ? SpeakerIds.kBlueSpeakerAprilTag2Id : SpeakerIds.kRedSpeakerAprilTag2Id;

    // Manual override
    speakerTag1Id = Constants.SpeakerConstants.kBlueSpeakerAprilTag1Id;
    speakerTag2Id = Constants.SpeakerConstants.kBlueSpeakerAprilTag2Id;

    speakerTag1Info = new TagInfo(speakerTag1Id);
    speakerTag2Info = new TagInfo(speakerTag2Id);
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
    speakerTag1Info.update();
    speakerTag2Info.update();
    speakerTag1Info.getEntry().setDouble(speakerTag1Info.getPos().getEntryZ());
    speakerTag2Info.getEntry().setDouble(speakerTag2Info.getPos().getEntryZ());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
