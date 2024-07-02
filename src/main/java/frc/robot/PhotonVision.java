// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class PhotonVision {
    
    PhotonCamera camera = new PhotonCamera("FrontCamera");
    AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d camerapose = new Transform3d();
    PhotonPoseEstimator PoseEstimator = new PhotonPoseEstimator(fieldLayout,PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,camera,camerapose);
    
    public static double GettargetDistance(){
        return 0;
    }

}
