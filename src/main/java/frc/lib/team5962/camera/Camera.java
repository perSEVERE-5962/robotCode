// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team5962.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  UsbCamera camera1;
  //  UsbCamera camera2;
  VideoSink server;

  public Camera() {

    camera1 = CameraServer.startAutomaticCapture(0);
    //    camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();
    camera1.setBrightness(50);
    camera1.setResolution(640, 480);
    //    camera2.setBrightness(50);
    //    camera2.setResolution(640, 480);
    server.setSource(camera1);
    SmartDashboard.putNumber("Camera", 1);
  }

  public void setBrightness(int brightness) {
    camera1.setBrightness(brightness);
  }

  public void ActivateCamera1() {
    SmartDashboard.putNumber("Camera", 1);
    server.setSource(camera1);
  }
  /*
  public void ActivateCamera2() {
    SmartDashboard.putNumber("Camera", 2);
    server.setSource(camera2);
  }
  */
}
