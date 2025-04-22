package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.PhotonVision;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;

public class AprilTags extends SubsystemBase {
  private static AprilTags instance;
  private AprilTags aprilTags;
  private static AddressableLED m_led;
  private static boolean atPosition;
  private static AddressableLEDBuffer m_ledBuffer;
  private static boolean targetVisible = false;
  public static Translation2d translation2d_2 = new Translation2d(0, 0);
  public static Rotation2d rotation2d_2 = new Rotation2d(0);
  public static Transform2d poseTransform2d_2 = new Transform2d(translation2d_2, rotation2d_2);
  public static Transform2d poseTransform2d_1 = new Transform2d(translation2d_2, rotation2d_2);
  public SwerveSubsystem cart = SwerveSubsystem.getInstance();
  public double angleToTagForCameraOne = 0;
  public double angleToTagForCameraTwo = 0;
  public int apriltagNumberCamera=0;
  public int apriltagNumberComparer=0;
public boolean isApriltagtheCorrectnumber=false;
  private AprilTags() {
    createLED();
  }

  public void periodic() {
     PhotonPipelineResult results = PhotonVisionConstant.CameraNames[1].getLatestResult();
    PhotonPipelineResult results2 = PhotonVisionConstant.CameraNames[3].getLatestResult();
    List<PhotonTrackedTarget> targets = results.getTargets();
    List<PhotonTrackedTarget> targets2 = results2.getTargets();
    targets.sort(PhotonTargetSortMode.Highest.getComparator());
    targets2.sort(PhotonTargetSortMode.Highest.getComparator());
    SmartDashboard.putBoolean("Camera 1 found Target", !targets.isEmpty());
    SmartDashboard.putBoolean("Camera 2 found Target", !targets2.isEmpty());
     int indexforTagOne=0;
     int indexforTagTwo=0;
    targetVisible = false;
    isApriltagtheCorrectnumber=false;
    for(int x=0;x<targets.size();x++){
      if(targets.get(x).getFiducialId()==apriltagNumberComparer){
        indexforTagOne=x;
        isApriltagtheCorrectnumber=true;
        break;
      }
    }
    for(int x=0;x<targets2.size();x++){
      if(targets2.get(x).getFiducialId()==apriltagNumberComparer){
        indexforTagTwo=x;
        isApriltagtheCorrectnumber=true;
        break;
      }
    }

    if (!targets.isEmpty() || !targets2.isEmpty()) {
      if (!targets.isEmpty()) {
        Transform3d targetYaw = targets.get(indexforTagOne).getBestCameraToTarget();
        poseTransform2d_2 = PhotonVision.transform3dtoTransform2d(targetYaw, false);
        targetVisible = true;
        apriltagNumberCamera=targets.get(indexforTagOne).getFiducialId();

      } else if (!targets2.isEmpty()) {
        Transform3d targetYaw = targets2.get(indexforTagTwo).getBestCameraToTarget();
        poseTransform2d_2 = PhotonVision.transform3dtoTransform2d(targetYaw, true);
        targetVisible = true;
        apriltagNumberCamera=targets2.get(indexforTagTwo).getFiducialId();
      }

    } else if (!targets.isEmpty() && !targets2.isEmpty()) {
      targetVisible = true;
      if (targets.get(0).getFiducialId() != targets2.get(0).getFiducialId()
          && targets.get(0).getArea() < targets2.get(0).getArea()) {

        Transform3d targetYaw = targets2.get(indexforTagTwo).getBestCameraToTarget();
        poseTransform2d_2 = PhotonVision.transform3dtoTransform2d(targetYaw, true);
        targetVisible = true;
        apriltagNumberCamera=targets2.get(indexforTagTwo).getFiducialId();
      } else if (targets.get(0).getFiducialId() != targets2.get(0).getFiducialId()
          && targets.get(0).getArea() > targets2.get(0).getArea()) {
        Transform3d targetYaw = targets.get(indexforTagOne).getBestCameraToTarget();
        poseTransform2d_2 = PhotonVision.transform3dtoTransform2d(targetYaw, false);
        targetVisible = true;
        apriltagNumberCamera=targets.get(indexforTagOne).getFiducialId();

      } else if (targets.get(0).getFiducialId() == targets2.get(0).getFiducialId()) {
        // Change to do averages
        Transform3d targetYaw = targets.get(indexforTagOne).getBestCameraToTarget();
        Transform3d targetYaw2 = targets2.get(indexforTagTwo).getBestCameraToTarget();
        poseTransform2d_2 = PhotonVision.transform3dtoTransform2d(targetYaw, false);
        poseTransform2d_1 = PhotonVision.transform3dtoTransform2d(targetYaw2, true);
        translation2d_2 = new Translation2d((poseTransform2d_2.getX() + poseTransform2d_1.getX()) / 2,
            (poseTransform2d_2.getY() + poseTransform2d_1.getY()) / 2);
            rotation2d_2 = new Rotation2d(poseTransform2d_2.getRotation().toMatrix());
        poseTransform2d_2 = new Transform2d(translation2d_2, rotation2d_2);
        targetVisible = true;
        translation2d_2 = new Translation2d(0, 0);
        apriltagNumberCamera=targets2.get(0).getFiducialId();
      }


    } 
 System.out.println(getObservations( PhotonVisionConstant.CameraNames[0],PhotonVisionConstant.FrontLeft.cameraposeFrontLeft).toString());

    setLED();
  }

  private void createLED() {
    m_led = new AddressableLED(0); // 0 = number of port on three letter thing i forgot what it called
    m_ledBuffer = new AddressableLEDBuffer(28); // 1 = number of leds in length of it
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    setLED();
  }

  public void isAtPosition(boolean atPosition) {
    this.atPosition = atPosition;
  }
  public int getFirstTagSeen(){
    return apriltagNumberCamera;
  }
  public void setFirstTagSeen(int x){
     this.apriltagNumberComparer=x;
  }
 

  private void setLED() {
    int hue = 0;
    if (targetVisible && !atPosition) {
      hue = ColorConstants.BlueHue;
    } else if (atPosition) {
      hue = ColorConstants.GreenHue;
    } else {
      hue = ColorConstants.RedHue;
    }

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, 255, 255); // could also do .setRGB if we want that color system
    }

    m_led.setData(m_ledBuffer);
  }

  public Transform2d getPosTogoTo() {
   
    return poseTransform2d_2;
  }

  public double getXPos() {
    return cart.getPose().getX();
  }

  public boolean getTargetVisabile() {
    return targetVisible;
  }

  public static AprilTags getInstance() {
    if (instance == null) {
      instance = new AprilTags();
    }

    return instance;
  }
 
public Pose3d getObservations(PhotonCamera camera, Transform3d robotToCamera) {
  PhotonPipelineResult results2 = camera.getLatestResult();
  List<PhotonTrackedTarget> targets = results2.getTargets();

        for (PhotonTrackedTarget result : targets) {
      
            

             // Single tag result
                var target = result;
            
                // Calculate robot pose
                var tagPose = Constants.PhotonVisionConstant.fieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                            tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                    return robotPose;

                   
                
              }
            }
        
                    return new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation());
                
}
}