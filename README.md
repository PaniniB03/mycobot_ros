# mycobot_ros
***This repo is a work in progress! The files are individually imported from Linux, not an accurate representation of the file hierarchy***

## Overview 

camera-calibration.py is to calibrate the camera on the end-effector
- the images are for calibration- all are used.

usb_cam-stream.launch is the ROS launch file to init nodes

panini_masterfollower_1.py is the main script for tracking the marker.
- it uses a script from an online tutorial but it is poorly applied as tracking is not reliable.


panini_masterfollower_2.py is a work in progress and derived from panini_masterfollower_1.py
- this is the main script being developed.
- Current steps involve using kinematic transformations to accurately "look" at the Aruco Marker


