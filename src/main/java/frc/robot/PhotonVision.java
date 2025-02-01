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
import frc.robot.Constants.PhotonVisionConstant;


import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PhotonVision {
    private int placeInArray;
    private Transform3d cameraPose; 
    private Transform3d  cameraPoses[]=new Transform3d[4];//4 is the number of Cameras
    public PhotonVision(){
    }

   private static PhotonCamera cameraFront = new PhotonCamera("FrontCamera");
   PhotonPipelineResult result=cameraFront.getLatestResult();
   private static PhotonCamera cameraSront = new PhotonCamera("Frontzmera");
   PhotonPipelineResult result1=cameraSront.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    List<PhotonTrackedTarget> targets1 = result1.getTargets();
   // PhotonTrackedTarget target= result.getBestTarget();
    static Transform3d camerapose = new Transform3d(new Translation3d(0,0,0),new Rotation3d(0,0,0));
    //Pose3d robotPose=PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),fieldLayout.getTagPose(target.getFiducialId()).get(), camerapose);
    
    public static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(PhotonVisionConstant.fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,camerapose);
 public Pose2d getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
 
        poseEstimator.setLastPose(prevEstimatedRobotPose);
        EstimatedRobotPose estimatedRobotPose= poseEstimator.update(result).orElse(null);
        EstimatedRobotPose estimatedRobotPose1= poseEstimator.update(result1).orElse(null);
        if(estimatedRobotPose==null && estimatedRobotPose1==null){
            return prevEstimatedRobotPose;
        }
        Rotation2d rotation2d = new Rotation2d(estimatedRobotPose.estimatedPose.getRotation().getX(),estimatedRobotPose.estimatedPose.getRotation().getY());
        Translation2d translation2d= new Translation2d(estimatedRobotPose.estimatedPose.getX(),estimatedRobotPose.estimatedPose.getY());
        Pose2d mapPose2d=new Pose2d(translation2d,rotation2d);
        return mapPose2d;
    }

     PhotonPoseEstimator PoseEstimator = new PhotonPoseEstimator(fieldLayout,PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE,camerapose);
    // public static double getTargetDistance() {
    //     if (cameraFront == null || !cameraFront.isConnected()) { return 0; }
    //  public static double GettargetDistance(){
    //      double distance = cameraFront.getCameraTable().getEntry("targetPose").getDoubleArray(new double[] {0,0,0})[0];
    //      System.out.println(distance);
    //     return distance;
    // }

    //     var result = cameraFront.getLatestResult();
    //     if (result.hasTargets()) {
    //         var bestTarget = result.getBestTarget();
    //         double range = PhotonUtils.calculateDistanceToTargetMeters(
    //             Units.inchesToMeters(Constants.CameraConstants.kCameraHeightInches),
    //             Units.inchesToMeters(Constants.CameraConstants.kCameraTargetHeightInches),
    //             Units.degreesToRadians(Constants.CameraConstants.kCameraPitchDegrees),
    //             Units.degreesToRadians(bestTarget.getPitch()));
    //         return range;
    //     }
    //     return 0;
    // }
}