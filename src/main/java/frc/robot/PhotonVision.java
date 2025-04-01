// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonTargetSortMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonVisionConstant;

/** Add your docs here. */
public class PhotonVision {
   //static PhotonPipelineResult result=PhotonVisionConstant.CameraNames[0].getLatestResult();
//   static PhotonPipelineResult result1=PhotonVisionConstant.CameraNames[1].getLatestResult();
//   static PhotonPipelineResult result2=PhotonVisionConstant.CameraNames[2].getLatestResult();
   static PhotonPipelineResult result3=PhotonVisionConstant.CameraNames[3].getLatestResult();
   

   public boolean targetVisible = false;
 public double targetYaw = 0.0;
 public double targetRange = 0.0;
 
  //  static List<PhotonTrackedTarget> targets = result.getTargets();
//     static List<PhotonTrackedTarget> targets1 = result1.getTargets();
//     static List<PhotonTrackedTarget> targets2 = result2.getTargets();
     static List<PhotonTrackedTarget> targets3 = result3.getTargets();
//    // PhotonTrackedTarget target= result.getBestTarget();
//     //static Transform3d camerapose = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
//Pose3d robotPose=PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),fieldLayout.getTagPose(target.getFiducialId()).get(), camerapose);
   
//     public static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(PhotonVisionConstant.fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,PhotonVisionConstant.CameraPoses[0]);
//     public static PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(PhotonVisionConstant.fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,PhotonVisionConstant.CameraPoses[1]);
//     public static PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(PhotonVisionConstant.fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,PhotonVisionConstant.CameraPoses[2]);
//     public static PhotonPoseEstimator poseEstimator3 = new PhotonPoseEstimator(PhotonVisionConstant.fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,PhotonVisionConstant.CameraPoses[3]);

//     public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
 
//         poseEstimator.setLastPose(prevEstimatedRobotPose);
//         poseEstimator1.setLastPose(prevEstimatedRobotPose);
//         poseEstimator2.setLastPose(prevEstimatedRobotPose);
//         poseEstimator3.setLastPose(prevEstimatedRobotPose);
//         EstimatedRobotPose estimatedRobotPose= poseEstimator.update(result).orElse(null);
//         EstimatedRobotPose estimatedRobotPose1= poseEstimator1.update(result1).orElse(null);
//         EstimatedRobotPose estimatedRobotPose2= poseEstimator2.update(result).orElse(null);
//         EstimatedRobotPose estimatedRobotPose3= poseEstimator3.update(result1).orElse(null);
//         if(estimatedRobotPose==null && estimatedRobotPose1==null && estimatedRobotPose2==null &&estimatedRobotPose3==null){
//             return prevEstimatedRobotPose;
//         }

//         Rotation2d rotation2d = new Rotation2d(
// (estimatedRobotPose.estimatedPose.getRotation().getX()+estimatedRobotPose1.estimatedPose.getRotation().getX()
// /*+estimatedRobotPose2.estimatedPose.getRotation().getX()+estimatedRobotPose3.estimatedPose.getRotation().getX()*/)/2,
// (estimatedRobotPose.estimatedPose.getRotation().getY()+estimatedRobotPose1.estimatedPose.getRotation().getY()
// /*+estimatedRobotPose2.estimatedPose.getRotation().getY()+estimatedRobotPose3.estimatedPose.getRotation().getY()*/)/2);
//         Translation2d translation2d= new Translation2d(
// (estimatedRobotPose.estimatedPose.getX()+estimatedRobotPose1.estimatedPose.getX()
// /*+estimatedRobotPose2.estimatedPose.getX()+estimatedRobotPose3.estimatedPose.getX()*/)/2
// /*PhotonVisionConstant.CameraNames.length*/,
// (estimatedRobotPose.estimatedPose.getY()+estimatedRobotPose1.estimatedPose.getY()
// /*+estimatedRobotPose2.estimatedPose.getY()+estimatedRobotPose3.estimatedPose.getY()*/)/2
// /*PhotonVisionConstant.CameraNames.length*/);
//         Pose2d mapPose2d=new Pose2d(translation2d,rotation2d);
//         return mapPose2d;

//     }

//  public static double getTargetDistance() {
//      if (cameraFront == null || !cameraFront.isConnected()) { return 0; }

//          var result = cameraFront.getLatestResult();
//          if (result.hasTargets()) {
//              var bestTarget = result.getTargetDistance();
//              double range = PhotonUtils.calculateDistanceToTargetMeters(
//                 Units.inchesToMeters(Constants.CameraConstants.kCameraHeightInches),
//                 Units.inchesToMeters(Constants.CameraConstants.kCameraTargetHeightInches),
//                  Units.degreesToRadians(Constants.CameraConstants.kCameraPitchDegrees),
//                Units.degreesToRadians(bestTarget.getPitch()));
//              return range;
//          }
//          return 0;
//       }
      public static Transform2d transform3dtoTransform2d(Transform3d pose,boolean isRight){

          
          double x = pose.getX();
          double y;
          if(isRight){
             y = pose.getY()+Constants.PhotonVisionConstant.YoffsetsForCameras-Constants.PhotonVisionConstant.kArmOffset;
          }else{
          y =  pose.getY()-Constants.PhotonVisionConstant.YoffsetsForCameras-Constants.PhotonVisionConstant.kArmOffset;
          }
   
           Translation2d translation2d = new Translation2d(x, y);
           Rotation2d rotation2d = pose.getRotation().toRotation2d();
           Transform2d poseTransform2d=new Transform2d(translation2d, rotation2d);
           System.out.println(y);
        

           return poseTransform2d;
           
           
      }
 
      public static Transform2d transform3dtoTransform2d(Transform3d pose){

          
         double x = pose.getX();
         double y =  pose.getY();
         
  
          Translation2d translation2d = new Translation2d(x, y);
          Rotation2d rotation2d = pose.getRotation().toRotation2d();
          Transform2d poseTransform2d=new Transform2d(translation2d, rotation2d);
     
       

          return poseTransform2d;
     }
}