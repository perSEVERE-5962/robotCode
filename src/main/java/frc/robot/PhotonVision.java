// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


/** Add your docs here. */
public class PhotonVision {
   private static PhotonCamera cameraFront = new PhotonCamera("FrontCamera");
    static AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPipelineResult result=cameraFront.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
   // PhotonTrackedTarget target= result.getBestTarget();
    static Transform3d camerapose = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
    //Pose3d robotPose=PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),fieldLayout.getTagPose(target.getFiducialId()).get(), camerapose);
    
    public static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,cameraFront,camerapose);
 public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
 
        poseEstimator.setLastPose(prevEstimatedRobotPose);
        EstimatedRobotPose estimatedRobotPose= poseEstimator.update().orElse(null);
        if(estimatedRobotPose==null){
            return prevEstimatedRobotPose;
        }
        Rotation2d rotation2d = new Rotation2d(estimatedRobotPose.estimatedPose.getRotation().getX(),estimatedRobotPose.estimatedPose.getRotation().getY());
        Translation2d translation2d= new Translation2d(estimatedRobotPose.estimatedPose.getX(),estimatedRobotPose.estimatedPose.getY());
        Pose2d mapPose2d=new Pose2d(translation2d,rotation2d);
        return mapPose2d;
    }

    // public static double GettargetDistance(){
    //     double distance = cameraFront.getCameraTable().getEntry("targetPose").getDoubleArray(new double[] {0,0,0})[0];
    //     System.out.println(distance);
    //     return distance;
    // }

}