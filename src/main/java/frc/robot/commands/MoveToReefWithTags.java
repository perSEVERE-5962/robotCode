// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.PhotonVision;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToReefWithTags extends Command {
  private boolean isRightPost = false;
  private AprilTags aprilTags;
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();

  /** Creates a new MoveToCoralStationWithTags. */
  public MoveToReefWithTags(boolean isRightPost) {
      addRequirements(SwerveSubsystem.getInstance());
      addRequirements(AprilTags.getInstance());
      this.aprilTags=AprilTags.getInstance();
      
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(aprilTags.getPosTogoTo().getX()*Math.sin(Math.abs(aprilTags.getPosTogoTo().getX()))>Constants.PhotonVisionConstant.kTargetXPos){

      driveTrain.move(-0.6, 0, 0);
    }


  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if ( aprilTags.getPosTogoTo().getX()<= Constants.PhotonVisionConstant.kTargetXPos){
      return true;
    }
    else{
    return false;
    }
  }
}
