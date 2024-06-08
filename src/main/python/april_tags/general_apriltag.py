#!/usr/bin/env python3

from networktables import NetworkTables
import dt_apriltags as ap
import cv2 as camera
import numpy as np
import sys

def meters_to_inches(meters: float) -> float:
    return meters * 0.0254

def main():
    ### Camera init ###

    capture = camera.VideoCapture(index=0) # Open the camera

    if not capture.isOpened():
        print("[ERROR] Camera failed to open")
        capture.release()
        NetworkTables.getDefault().getTable("apriltags").getEntry("cameraworking").setBoolean(False)
        return

    focal_data = (532, 502, 320, 240) # Brute forced, most accurate so far and most accurate at 4 yards
    tag_size = meters_to_inches(6.5)

    ### Networktable init ###

    # Full paths (For Java)
    ## NetworkTableInstance.getDefault().getTable("apriltags") -> Master table
    # General
    ## NetworkTableInstance.getDefault().getTable("apriltags").getEntry("cameraworking")
    ## NetworkTableInstance.getDefault().getTable("apriltags").getEntry("foundtag")
    # Pose estimation
    ## NetworkTableInstance.getDefault().getTable("apriltags").getSubTable("pos").getEntry("x/y/z")
    ## NetworkTableInstance.getDefault().getTable("apriltags").getEntry("angletotag")

    if len(sys.argv) >= 2:
        NetworkTables.initialize(sys.argv[1])
        print("Initialized as client")
    else:
        NetworkTables.initialize()
        print("Initialized as server")

    apriltags_networktable = NetworkTables.getDefault().getTable("apriltags")
    apriltags_networktable.getEntry("cameraworking").setBoolean(True)

    ### Apriltag init ###

    detector = ap.Detector(families="tag36h11", nthreads=2, quad_decimate=1.0) # This takes an image and finds apriltags
    pos_subtable = apriltags_networktable.getSubTable("pos") # Position storage

    ### Main program loop ###

    #video = camera.VideoWriter("capture.avi", camera.VideoWriter_fourcc(*'MJPG'), 10, (640, 480))

    while True:
        exists, frame = capture.read() # Grab a frame off the camera stream
        if exists:
            image = camera.cvtColor(frame, camera.COLOR_BGR2GRAY) # Convert the frame to a grayscale image. Apriltag detection only works with grayscale images.
            #video.write(image)
            results = detector.detect(img=image, estimate_tag_pose=True, camera_params=focal_data, tag_size=tag_size)

            if len(results) > 0: # Found a tag
                apriltags_networktable.getEntry("foundtag").setBoolean(True)
                for tag in results:
                    apriltags_networktable.getEntry("id").setNumber(tag.tag_id)

                    pose_translation = tag.pose_t
                    pos_x = pose_translation[0] # Left/right
                    pos_y = pose_translation[1] # Up/down
                    pos_z = pose_translation[2] # Forwards/backwards / Distance

                    pos_subtable.getEntry("x").setNumber(pos_x)
                    pos_subtable.getEntry("y").setNumber(pos_y)
                    pos_subtable.getEntry("z").setNumber(pos_z)

                    apriltags_networktable.getEntry("angletotag").setNumber(np.arctan2(pos_x, pos_z))
            else: # Did not find a tag
                apriltags_networktable.getEntry("foundtag").setBoolean(False)

main()