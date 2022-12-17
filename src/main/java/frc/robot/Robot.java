// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Callable;

import ROS_Interface.msgs.LaserScanMsg;
import ROS_Interface.src.NodeHandle;
import ROS_Interface.src.Subscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private NodeHandle nh;
  private Callable<Void> callback;
  private NetworkTableEntry laser_topic;
  private DriveTrain m_driveTrain;
  private boolean processing = false;

  public float getleft(LaserScanMsg msg){
    for(int i=0; i < msg.ranges_size; ++i){
      if(!Float.isNaN(msg.ranges[i]))
        return msg.ranges[i];
    }
    return Float.NaN;
  }

  public float getRight(LaserScanMsg msg){
    for(int i=msg.ranges_size-1; i > 0; --i){
      if(!Float.isNaN(msg.ranges[i]))
        return msg.ranges[i];
    }
    return Float.NaN;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = RobotContainer.getInstance();

    // Ignore this for now
//    callback = new Callable<Void>() {
//      @Override
//      public Void call() throws Exception {
//        laser_scan_callback();
//        return null;
//      }
//    };


    //ROS Stuff
    this.nh = new NodeHandle();
    Subscriber laser_sub = nh.subscribe("laser_scan", callback);


    //Ignore this
    laser_topic = laser_sub.getTopic();
    }

  public void laser_scan_callback(){
    processing = true;
    //Ignore this
    byte[] raw_msg = laser_topic.getRaw(new byte[0]);
    LaserScanMsg msg = new LaserScanMsg(raw_msg);

    float left_value = getleft(msg);
    SmartDashboard.putNumber("LEFT_VALUE", left_value);
    float right_value = getRight(msg);
    SmartDashboard.putNumber("RIGHT_VALUE", right_value);

    float error = Math.abs(left_value - right_value);
    
    
    //MAKE CHANGES HERE!
    double TOLERANCE = 0.1;
    if(error < TOLERANCE){
      SmartDashboard.putString("COMMAND", "STOP");
      m_driveTrain.stopDrive();
    }
    else if(left_value > right_value){
      SmartDashboard.putString("COMMAND", "TURN1");
      m_driveTrain.arcadeDrive(0, 0.1);
    }
    else if(right_value > left_value){
      SmartDashboard.putString("COMMAND", "TURN2");
      m_driveTrain.arcadeDrive(0, 0.1);
    }
    processing = false;
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
    m_robotContainer.setMotorControllerType();
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
    Joystick joystick = m_robotContainer.getJoystick();
    m_robotContainer.getDriveTrain().arcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(4)*-1);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setMotorControllerType();
    m_driveTrain = RobotContainer.getInstance().getDriveTrain();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(!processing)
      laser_scan_callback();
  }
}
