// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image and sent to the dashboard.
 *
 * <p>Be aware that the performance on this is much worse than a coprocessor solution!
 */
public class DetectAprilTags {
  //static double[][] posArray = new double[10][3];
  //static double[][] rotArray = new double[10][3];
  private static ArrayList<double[]> posArray = new ArrayList<>();
  private static ArrayList<double[]> rotArray = new ArrayList<>();
  public static int amountOfDetections = 0;
  private static ArrayList<Integer> tagId = new ArrayList<>();

  public void initDetector() {
    var visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();
    // look for tag16h5, don't correct any error bits
    detector.addFamily("tag36h11", 0);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    var poseEstConfig =
        new AprilTagPoseEstimator.Config(
            0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    // We'll output to NT
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      amountOfDetections = detections.length;

      // have not seen any tags yet
      tags.clear();

      tagId.clear();
      posArray.clear();
      rotArray.clear();
      for (AprilTagDetection detection : detections) {
        // remember we saw this tag
        tags.add((long) detection.getId());
        tagId.add(detection.getId());

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Integer.toString(detection.getId()),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);

        // Determine pose
        Transform3d pose = estimator.estimate(detection);
        // Get rotation
        Rotation3d rot = pose.getRotation();

        // Set the values to their respective arrays
        double[] positions = {pose.getX(), pose.getY(), pose.getZ()};
        double[] rotations = {rot.getX(), rot.getY(), rot.getZ()};
        posArray.add(positions);
        rotArray.add(rotations);
      }

      // put list of tags onto dashboard
      pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    pubTags.close();
    detector.close();
  }

  /**
   * Pos 0 (X): Left/right
   *
   * <p>Pos 1 (Y): Up/down
   *
   * <p>Pos 2 (Z): Forward/backward
   * 
   * @return The position of the april tag index specified, or null if such april tag doesn't exist.
   */
  public static double[] getAprilTagPos(int index) {
    posArray.trimToSize();
    if (index > posArray.size() - 1) {
      return null;
    }
    return posArray.get(index);
  }

  /**
   * Rot 0 (Pitch)
   *
   * <p>Rot 1 (Yaw)
   *
   * <p>Rot 2 (Roll)
   * 
   * @return The rotation of the april tag index specified, or null if such april tag doesn't exist.
   */
  public static double[] getAprilTagRot(int index) {
    rotArray.trimToSize();
    if (index > rotArray.size() - 1) {
      return null;
    }
    return rotArray.get(index);
  }

  public static Integer getAprilTagId(int index) {
    tagId.trimToSize();
    if (index > tagId.size() - 1) {
      return null;
    }
    return tagId.get(index);
  }

  public static void displayAprilTagInformation() {
    for (int i = 0; i < amountOfDetections; i++) {
      double[] pos = getAprilTagPos(i);
      double[] rotPos = getAprilTagRot(i);
      Integer id = getAprilTagId(i);
      NetworkTableEntry entryX = NetworkTableInstance.getDefault().getEntry("Tag Pos X: " + i);
      NetworkTableEntry entryY = NetworkTableInstance.getDefault().getEntry("Tag Pos Y: " + i);
      NetworkTableEntry entryZ = NetworkTableInstance.getDefault().getEntry("Tag Pos Z: " + i);
      NetworkTableEntry entryRotX = NetworkTableInstance.getDefault().getEntry("Tag Rot X: " + i);
      NetworkTableEntry entryRotY = NetworkTableInstance.getDefault().getEntry("Tag Rot Y: " + i);
      NetworkTableEntry entryRotZ = NetworkTableInstance.getDefault().getEntry("Tag Rot Z: " + i);
      NetworkTableEntry angle = NetworkTableInstance.getDefault().getEntry("Angle to tag: " + i);
      NetworkTableEntry ID = NetworkTableInstance.getDefault().getEntry("Tag ID: " + i);
      NetworkTableEntry count = NetworkTableInstance.getDefault().getEntry("Tag count");
      if (pos != null && rotPos != null && id != null) {
        count.setInteger(amountOfDetections);
        ID.setInteger(id);
        entryX.setDouble(pos[0]);
        entryY.setDouble(pos[1]);
        entryZ.setDouble(pos[2]);
        entryRotX.setDouble(rotPos[0]);
        entryRotY.setDouble(rotPos[1]);
        entryRotZ.setDouble(rotPos[2]);
        angle.setDouble(Math.toDegrees(Math.atan2(pos[0], pos[2])));
      }
    }
  }
}