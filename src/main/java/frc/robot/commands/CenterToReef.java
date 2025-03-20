// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.Constants.StartingPos;
import frc.robot.PhotonVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterToReef extends Command {
  private boolean targetVisible=false;
  /** Creates a new CenterToReef. */
  public CenterToReef() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d pose=new Transform3d();

    for (int i = 0; i < 10; i++) {
      var results = PhotonVisionConstant.CameraNames[1].getAllUnreadResults();//1 before
      if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets() ) {
          // At least one AprilTag was seen by the camera
   
          double x=SmartDashboard.getNumber("Auto", 1);
     SmartDashboard.putNumber("Auto-Selected", x);
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
          for (var target : result.getTargets()) {
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
    SmartDashboard.putNumber("y", pose.getY());
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //
  }
}
