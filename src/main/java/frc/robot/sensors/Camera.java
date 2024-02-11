package frc.robot.sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;

public class Camera {
    private VideoSink server;
    private UsbCamera camera;
    public Camera(int port){
        camera = CameraServer.startAutomaticCapture(port);
        camera.setResolution(480, 320);
        server = CameraServer.getServer();
        server.setSource(camera);
    }
    public void setBrightness(int brightness){
        camera.setBrightness(brightness);
    }
    
}
