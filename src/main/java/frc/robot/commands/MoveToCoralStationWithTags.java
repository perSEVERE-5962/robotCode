// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.PhotonVision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToCoralStationWithTags extends Command {
 private boolean targetVisible =false;
 private double targetYaw = 0.0;
 private double targetRange = 0.0;
 private boolean targetVisible2 =false;
 private double targetYaw2 = 0.0;
 private double targetRange2 = 0.0;
 private double area=0.0;
 private boolean isRightPost = false;
 
  /** Creates a new MoveToCoralStationWithTags. */
  public MoveToCoralStationWithTags(boolean isRightPost) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var results = PhotonVisionConstant.CameraNames[1].getAllUnreadResults();
    var results2 = PhotonVisionConstant.CameraNames[0].getAllUnreadResults();
    
    if (!results.isEmpty()||!results2.isEmpty()) {
     // Camera processed a new frame since last
     // Get the last one in the list.
     var result = results.get(results.size() - 1);
     var result2 = results2.get(results.size() - 1);
     if (result.hasTargets()||result2.hasTargets()) {
         // At least one AprilTag was seen by the camera
         for (var target : result.getTargets()) {
             if (target.getFiducialId() == 9 || target.getFiducialId() == 6) {
                 // Found Tag 7, record its information
                 if(target.getArea()>area){
                 targetYaw = target.getYaw();
                 targetRange =
                         PhotonUtils.calculateDistanceToTargetMeters(
                                 0.5, // Measured with a tape measure, or in CAD.
                                 0.3, // From 2024 game manual for ID 7
                                 Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                 Units.degreesToRadians(target.getPitch()));
                 
 
                 targetVisible = true;
                 area=target.getArea();
                 }


             }
             for (var target2 : result.getTargets()) {
              if (target2.getFiducialId() == 9 || target2.getFiducialId() == 6) {
                  // Found Tag 7, record its information
                  if(target2.getArea()>area){
                  targetYaw = target.getYaw();
                  targetRange =
                          PhotonUtils.calculateDistanceToTargetMeters(
                                  0.5, // Measured with a tape measure, or in CAD.
                                  0.3, // From 2024 game manual for ID 7
                                  Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                  Units.degreesToRadians(target.getPitch()));
                  
  
                  targetVisible = true;
                  area=target.getArea();
                  }
 
 
              }
         }
     }
 }
}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
   
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
