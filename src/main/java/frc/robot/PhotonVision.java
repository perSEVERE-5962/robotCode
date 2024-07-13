// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


/** Add your docs here. */
public class PhotonVision {
   private static PhotonCamera cameraFront = new PhotonCamera("FrontCamera");
    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPipelineResult result=cameraFront.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target= result.getBestTarget();
    Transform3d camerapose = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
    Pose3d robotPose=PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),fieldLayout.getTagPose(target.getFiducialId()).get(), camerapose);


    
    PhotonPoseEstimator PoseEstimator = new PhotonPoseEstimator(fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,cameraFront,camerapose);
    
    // public static double GettargetDistance(){
    //     double distance = cameraFront.getCameraTable().getEntry("targetPose").getDoubleArray(new double[] {0,0,0})[0];
    //     System.out.println(distance);
    //     return distance;
    // }

}