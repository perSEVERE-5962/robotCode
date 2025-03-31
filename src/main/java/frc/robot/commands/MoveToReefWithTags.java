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
  private AprilTags aprilTags;
  private final SwerveSubsystem driveTrain = SwerveSubsystem.getInstance();
  private boolean angleIsCorrect= false;
private double rotation=0;
private double x=0;
private double y=0;
private boolean yIsCorrect=false;
private double offest;

  /** Creates a new MoveToCoralStationWithTags. */
  public MoveToReefWithTags(boolean isRightPost) {
      addRequirements(SwerveSubsystem.getInstance());
      addRequirements(AprilTags.getInstance());
      this.aprilTags=AprilTags.getInstance();
      if(isRightPost==true){
        offest=Constants.PhotonVisionConstant.kTagToPost;
      }else{
        offest=-Constants.PhotonVisionConstant.kTagToPost;
      }
    
      
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yIsCorrect=false;
    angleIsCorrect= false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   double rotationmin=0;
    if(!(aprilTags.getPosTogoTo().getRotation().getRadians()<Math.PI && aprilTags.getPosTogoTo().getRotation().getRadians()>Math.toRadians(179.5) 
    ||(aprilTags.getPosTogoTo().getRotation().getRadians()>-Math.PI && aprilTags.getPosTogoTo().getRotation().getRadians()<Math.toRadians(-179.5) ))){
      if(aprilTags.getPosTogoTo().getRotation().getRadians()>0){
        rotation=-1;
        rotationmin=-0.1;
      }else {
        rotation=1;
        rotationmin=0.1;
      }
      angleIsCorrect= false;

    }else{
      rotation=0;
      angleIsCorrect= true;
    }
    if(aprilTags.getPosTogoTo().getX()>Constants.PhotonVisionConstant.kTargetXPos){

      x=0.4;
      
    }else{
      x=0;
    }
    double u=0;
     if(!(aprilTags.getPosTogoTo().getY()>-0.01+offest && aprilTags.getPosTogoTo().getY()<0.01+offest) ){
      if(aprilTags.getPosTogoTo().getY()<offest){
     u=0.1;

      }else{
        u=-0.1;
      }

    }else{
      yIsCorrect=true;
      y=0;
    }
    
    y=-(aprilTags.getPosTogoTo().getY())+offest*0.9*1+u;
    rotation=rotation*(Math.PI-Math.abs(aprilTags.getPosTogoTo().getRotation().getRadians()))*0.9*1+rotationmin;
    x=((aprilTags.getPosTogoTo().getX())-Constants.PhotonVisionConstant.kTargetXPos)*0.9*1;
    driveTrain.move(x, y,rotation);
    
    rotation=0;
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if ( aprilTags.getTargetVisabile()){
      if(/* aprilTags.getPosTogoTo().getX()<= Constants.PhotonVisionConstant.kTargetXPos && angleIsCorrect && */yIsCorrect ){
        return true;
      }
      aprilTags.isAtPosition(false);
      return false;
    }
    else{
    aprilTags.isAtPosition(false);
    return true;
    }
  }
}
