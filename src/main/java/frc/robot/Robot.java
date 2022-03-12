// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Command m_driveCommand;
  private Command m_intakeSpeed;
  private Command m_arm;
  private Command m_camerafeed; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

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

    SmartDashboard.putNumber(
        "Drive Encoder", m_robotContainer.getDriveTrain().getAverageEncoderDistance());
    SmartDashboard.putNumber("Arm Encoder", m_robotContainer.getArm().getPosition());
    SmartDashboard.putNumber("Gyro Angle", m_robotContainer.getDriveTrain().getGyroAngle());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorControllerType();
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

    m_robotContainer.setMotorControllerType();
    m_driveCommand = m_robotContainer.getDriveCommand();
    if (m_driveCommand != null) {
      m_driveCommand.schedule();
    }
    m_intakeSpeed = m_robotContainer.getIntakeSpeed();
    if (m_intakeSpeed != null) {
      m_intakeSpeed.schedule();
    }
    m_arm = m_robotContainer.getArmCommand();
    if (m_arm != null) {
      m_arm.schedule();
    }

  }

  /** This function is called periodically during operator control. 
   * @param Joy1 */
  @Override
  public void teleopPeriodic() {
    int brightness = (int) SmartDashboard.getNumber("Camera Brightness", 50);
    m_robotContainer.setCameraBrightness(brightness);
    if (m_robotContainer.getCopilotJoystick().getRawButtonPressed(1)){
      m_robotContainer.getCamera().ActivateCamera1();
    }
    else if (m_robotContainer.getCopilotJoystick().getRawButtonPressed(2)){
      m_robotContainer.getCamera().ActivateCamera2();
    }
    
  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
