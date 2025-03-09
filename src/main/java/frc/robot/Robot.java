// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.PhotonVision;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstant;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PhotonVision poseEstimator = new PhotonVision();
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  @Override
  public void driverStationConnected() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      Constants.kTeamColor = Constants.TEAM_COLOR_BLUE;
    } else {
      Constants.kTeamColor = Constants.TEAM_COLOR_RED;
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Translation2d startingTranslation2d=new
    // Translation2d(Constants.StartingPos.startingTranslation2dx,
    // Constants.StartingPos.startingTranslation2dy);
    // Rotation2d statingRotation2d=new
    // Rotation2d(Constants.StartingPos.statingRotation2dx,Constants.StartingPos.statingRotation2dy);
    // Pose2d startPose2d =new Pose2d(startingTranslation2d,statingRotation2d);
    // driveTrain.resetOdometry(startPose2d);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    Transform3d pose=new Transform3d();
    m_robotContainer = RobotContainer.getInstance();
    boolean targetVisible = false;
    for (int i = 0; i < 10; i++) {
      var results = PhotonVisionConstant.CameraNames[1].getAllUnreadResults();
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
          // At least one AprilTag was seen by the camera
          PhotonPipelineResult result1=PhotonVisionConstant.CameraNames[1].getLatestResult();
     PhotonVision.targets3 = result1.getTargets();
          for (var target : result.getTargets()) {
          //  var targetPose=target;
            if (target.getFiducialId() == 6) {
              pose=target.getBestCameraToTarget();
              Constants.StartingPos.poseTransform2d=PhotonVision.transform3dtoTransform2d(pose);
              // Found Tag 7, record its information
         //     poseTransform2d= target.getFiducialId().getCameraToTarget();
              targetVisible = true;

            }
          }
        }
      }
      if (targetVisible == true) {
        break;
      }

    }
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("x-pos", pose.getX());
    SmartDashboard.putNumber("y", pose.getY());

    SmartDashboard.putNumber("wristP", Constants.WristConstants.kP);
    SmartDashboard.putNumber("wristI", Constants.WristConstants.kI);
    SmartDashboard.putNumber("wristD", Constants.WristConstants.kD);
    SmartDashboard.putNumber("pivotP", Constants.PivotConstants.kP);
    SmartDashboard.putNumber("pivotI", Constants.PivotConstants.kI);
    SmartDashboard.putNumber("pivotD", Constants.PivotConstants.kD);
    SmartDashboard.putNumber("reachP", Constants.ReachConstants.kP);
    SmartDashboard.putNumber("reachI", Constants.ReachConstants.kI);
    SmartDashboard.putNumber("reachD", Constants.ReachConstants.kD);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // PhotonPipelineResult
    // result=PhotonVisionConstant.CameraNames[0].getLatestResult();
    // PhotonVision.targets = result.getTargets();
    // PhotonPipelineResult
    // result1=PhotonVisionConstant.CameraNames[1].getLatestResult();
    // PhotonVision.targets1 = result1.getTargets();
    // PhotonPipelineResult
    // result2=PhotonVisionConstant.CameraNames[2].getLatestResult();
    // PhotonVision.targets2 = result2.getTargets();
    // PhotonPipelineResult
    // result3=PhotonVisionConstant.CameraNames[3].getLatestResult();
    // PhotonVision.targets3 = result3.getTargets();
    // // if(!PhotonVision.targets.isEmpty() || !PhotonVision.targets1.isEmpty()||
    // !PhotonVision.targets2.isEmpty()|| !PhotonVision.targets3.isEmpty()){
    // //// Pose2d PoseEstimator
    // =poseEstimator.getEstimatedGlobalPose(m_robotContainer.getDrivetrainSubsystem().getPose());
    // // driveTrain.resetOdometry(PoseEstimator);
    // // System.out.print(PoseEstimator);
    // // }

    // // System.out.println(PhotonVision.targets1.isEmpty());

    // tag 21 if blue
    // Transform2d
    // distanceToTarget3=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets3,6);
    // Transform2d
    // distanceToTarget1=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets1,6);
    // Transform2d
    // distanceToTarget=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets,6);
    // Transform2d
    // distanceToTarget2=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets2,6);
    // tag 10 if red
    // Transform2d
    // distanceToTarget3tag10=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets3,10);
    // Transform2d
    // distanceToTarget1tag10=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets1,10);
    // Transform2d
    // distanceToTargettag10=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets,10);
    // Transform2d
    // distanceToTarget2tag10=PhotonVision.distanceToAprilTagForOneCamera(PhotonVision.targets2,10);
    // System.out.println(distanceToTarget2);
    // SmartDashboard.putNumber( "distancetotagx",distanceToTarget2.getX());
    // SmartDashboard.putNumber( "distancetotagy",distanceToTarget2.getY());

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SwerveSubsystem sss = SwerveSubsystem.getInstance();
    sss.outputEncoderPositions();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
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
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    if (m_autonomousCommand != null) {

      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
