/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

public class Camera extends SubsystemBase {
  /**
   * Creates a new Camera.
   */
  public static CvSink cvSink;
  public static CvSource cameraOutput;

  public Camera() {
    CameraServer.getInstance().startAutomaticCapture(1);
    //cvSink = CameraServer.getInstance().getVideo();
    //cameraOutput = CameraServer.getInstance().putVideo("DriverCamera", 1920, 1080);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
