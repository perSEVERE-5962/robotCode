// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.PhotonVision;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToReefWithTags extends Command {
  private boolean targetVisible = false;
  private double targetYaw = 0.0;
  private double targetRange = 0.0;
  private boolean isRightPost = false;
  public static Translation2d translation2d_2= new Translation2d(0, 0);
   public static Rotation2d rotation2d_2= new Rotation2d(0);
    public static Transform2d  poseTransform2d_2=new Transform2d(translation2d_2, rotation2d_2);

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
 private SwerveSubsystem cart= SwerveSubsystem.getInstance();
  /** Creates a new MoveToCoralStationWithTags. */
  public MoveToReefWithTags(boolean isRightPost) {
      addRequirements(SwerveSubsystem.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led = new AddressableLED(0); // 0 = number of port on three letter thing i forgot what it called
    m_ledBuffer = new AddressableLEDBuffer(9); // 1 = number of leds in length of it
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    setLED();
  }
  private void setLED(){
    int hue = 0;
    if (targetVisible) {
      hue = ColorConstants.BlueHue;
    } else {
      hue = ColorConstants.RedHue;
    }
  
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setHSV(i, hue, 255, 255);   // could also do .setRGB if we want that color system
    }
  
    m_led.setData(m_ledBuffer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetVisible = false;
    PhotonPipelineResult results = PhotonVisionConstant.CameraNames[1].getLatestResult();
    PhotonPipelineResult results2 = PhotonVisionConstant.CameraNames[2].getLatestResult();
    List<PhotonTrackedTarget> targets = results.getTargets();
    List<PhotonTrackedTarget> targets2 = results2.getTargets();
    targets.sort(PhotonTargetSortMode.Highest.getComparator());
    targets2.sort(PhotonTargetSortMode.Highest.getComparator());

    if (targets.isEmpty() || targets2.isEmpty()) {

      if (!targets.isEmpty()) {
       Transform3d targetYaw = targets.get(0).getBestCameraToTarget();;
       poseTransform2d_2=PhotonVision.transform3dtoTransform2d(targetYaw);
        targetVisible = true;
      } else if (!targets2.isEmpty()) {
        Transform3d targetYaw = targets.get(0).getBestCameraToTarget();;
       poseTransform2d_2=PhotonVision.transform3dtoTransform2d(targetYaw);
        targetVisible = true;
      }

    } else if (!targets.isEmpty() && !targets2.isEmpty()) {
      targetVisible = true;
      if (targets.get(0).getFiducialId() != targets2.get(0).getFiducialId()
          && targets.get(0).getArea() < targets2.get(0).getArea()) {
      
            Transform3d targetYaw = targets.get(0).getBestCameraToTarget();
            poseTransform2d_2=PhotonVision.transform3dtoTransform2d(targetYaw);
      } else if (targets.get(0).getFiducialId() != targets2.get(0).getFiducialId()
          && targets.get(0).getArea() > targets2.get(0).getArea()) {
        
            Transform3d targetYaw = targets.get(0).getBestCameraToTarget();;
            poseTransform2d_2=PhotonVision.transform3dtoTransform2d(targetYaw);
            
      } else if (targets.get(0).getFiducialId() == targets2.get(0).getFiducialId()) {
        // Change to do averages
       
        Transform3d targetYaw = targets.get(0).getBestCameraToTarget();;
       poseTransform2d_2=PhotonVision.transform3dtoTransform2d(targetYaw);
          
      }

    }
    setLED();
    if(targetVisible==true){

      
    }
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.stop();
    m_led.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( cart.getPose().getX()<= Constants.StartingPos.kTargetXPos){
      return true;
    }
    else{
    return false;
    }
  }
}
