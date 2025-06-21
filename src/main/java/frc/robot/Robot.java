// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.PhotonVision;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstant;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.Constants.StartingPos;
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
  //PWMSparkMax Pivot;
  private RobotContainer m_robotContainer;
  
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

 /*  public Robot() {
    enableLiveWindowInTest(true);
    Pivot = new PWMSparkMax(0);

    
} */


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
    m_robotContainer = RobotContainer.getInstance();
    

    SmartDashboard.putNumber("wristP", Constants.WristConstants.kP);
    SmartDashboard.putNumber("wristI", Constants.WristConstants.kI);
    SmartDashboard.putNumber("wristD", Constants.WristConstants.kD);
    SmartDashboard.putNumber("pivotP", Constants.PivotConstants.kP);
    SmartDashboard.putNumber("pivotI", Constants.PivotConstants.kI);
    SmartDashboard.putNumber("pivotD", Constants.PivotConstants.kD);
    SmartDashboard.putNumber("reachP", Constants.ReachConstants.kP);
    SmartDashboard.putNumber("reachI", Constants.ReachConstants.kI);
    SmartDashboard.putNumber("reachD", Constants.ReachConstants.kD);

    AprilTags.getInstance();
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
    
  
    CommandScheduler.getInstance().run();
   Pose3d x= AprilTags.getInstance().getObservations(PhotonVisionConstant.CameraNames[2],PhotonVisionConstant.CameraPoses[2]);
    SmartDashboard.putNumber("x-pose", x.getX());
    SmartDashboard.putNumber("y-pose", x.getY());
    SmartDashboard.putNumber("z-pose", x.getZ());
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
    /*Transform3d pose=new Transform3d();
    boolean targetVisible = false;

    for (int i = 0; i < 10; i++) {
      var results = PhotonVisionConstant.CameraNames[1].getAllUnreadResults();
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets() ) {
          // At least one AprilTag was seen by the camera
   
     double x=1;
     //SmartDashboard.putNumber("Auto-Selected", x);
          for (var target : result.getTargets()) {
          //  var targetPose=target;
          
            if (target.getFiducialId() == StartingPos.apriltagsToReef[(int)x][0] || target.getFiducialId() == StartingPos.apriltagsToReef[(int)x][1] ) {//First is which on you want to go to, the second one is the color you are.
              pose=target.getBestCameraToTarget();
              Constants.StartingPos.poseTransform2d=PhotonVision.transform3dtoTransform2d(pose);
              // Found Tag 7, record its information
         //     poseTransform2d= target.getFiducialId().getCameraToTarget();
              targetVisible = true;

            }
          }
          /* for (var target : resultsCameras.getTargets()) {
            //  var targetPose=target;
              if (target.getFiducialId() == StartingPos.apriltagsToReef[0][0] || target.getFiducialId() == StartingPos.apriltagsToReef[0][1] ) {//First is which on you want to go to, the second one is the color you are.
                pose=target.getBestCameraToTarget();
                Constants.StartingPos.poseTransform2d_2=PhotonVision.transform3dtoTransform2d(pose);
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
    SmartDashboard.putNumber("y", pose.getY());*/

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
