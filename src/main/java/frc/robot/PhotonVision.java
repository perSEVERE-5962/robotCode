// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PhotonVision {
    private static PhotonCamera cameraFront = new PhotonCamera("FrontCamera");

    public static double getTargetDistance() {
        if (cameraFront == null || !cameraFront.isConnected()) { return 0; }

        var result = cameraFront.getLatestResult();
        if (result.hasTargets()) {
            var bestTarget = result.getBestTarget();
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                Units.inchesToMeters(Constants.CameraConstants.kCameraHeightInches),
                Units.inchesToMeters(Constants.CameraConstants.kCameraTargetHeightInches),
                Units.degreesToRadians(Constants.CameraConstants.kCameraPitchDegrees),
                Units.degreesToRadians(bestTarget.getPitch()));
            return range;
        }
        return 0;
    }
}
