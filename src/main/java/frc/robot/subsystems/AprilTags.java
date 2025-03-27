package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.PhotonVisionConstant;
import frc.robot.PhotonVision;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;

public class AprilTags extends SubsystemBase {
    private static AprilTags instance;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean targetVisible = false;
    public static Translation2d translation2d_2= new Translation2d(0, 0);
    public static Rotation2d rotation2d_2= new Rotation2d(0);
    public static Transform2d  poseTransform2d_2=new Transform2d(translation2d_2, rotation2d_2);
    public SwerveSubsystem cart= SwerveSubsystem.getInstance();
    private AprilTags(){
        createLED();
    }

    public void periodic(){
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
    }

    private void createLED(){
        m_led = new AddressableLED(0); // 0 = number of port on three letter thing i forgot what it called
        m_ledBuffer = new AddressableLEDBuffer(18); // 1 = number of leds in length of it
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

  public double getXPos(){
    return cart.getPose().getX();
  }
  public static AprilTags getInstance() {
    if (instance == null) {
      instance = new AprilTags();
    }

    return instance;
  }

}