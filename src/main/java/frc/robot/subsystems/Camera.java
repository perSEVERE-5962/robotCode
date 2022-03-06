// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  UsbCamera camera1;
  VideoSink server;
  UsbCamera camera2; 
  Joystick joy1 = new Joystick(0); 
  public Camera() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    camera1.setBrightness(50);
    camera1.setResolution(480, 320); 
    camera2.setBrightness(50);
    camera2.setResolution(480, 320);

    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen); 

    if (joy1.getTriggerPressed()) {
      System.out.println("Setting Camera 2");
      server.setSource(camera2);
    } else if(joy1.getTriggerReleased()) {
      System.out.println("Setting Camera 1");
      server.setSource(camera1);
    }
  }
  public void setBrightness(int brightness) {
    camera1.setBrightness(brightness);
  }
}
