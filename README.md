# mycobot_ros
***This repo is a work in progress! The files are individually imported from Linux and is not an accurate representation of the file hierarchy.
This will be updated once the final tracking program is finished.***

In my linux environment, Elephant Robotic's *mycobot_ros* repo was cloned and the following scripts are based off of that package. It can be seen that there are many code segments from different scripts combined and modified for this project's use.

## Aruco Marker Detection

- *camera-calibration.py* is to calibrate a Logitech Brio Webcam on the end-effector.
	- All jpeg images were taken with the webcam and used for calibration purposes.
	- This returns a camera matrix, distortion coefficients, Rodrigues Vector (rvec), and translation vector (tvec).

- *usb_cam-stream.launch* is the ROS launch file to initialize ROS nodes for the webcam to operate.
- *detect.py* is the script to actively detect the Aruco Marker's 6-DOF Pose, relative to webcam. It is also easily modifiable to detect multiple markers if needed.
	- Demonstration:

<p align="center">
<img width="300" alt="image" src="https://github.com/user-attachments/assets/f30323f8-8c6d-4841-a793-9cb87279b52a">
</p>

https://github.com/user-attachments/assets/dd4433e9-6f7d-485a-b935-69614494542e


## Robot Control & Simulation

There are several ways to simulate it via Rviz, control the robot without Rviz, and control the robot with Rviz simulating real-time movement. The *mycobot_ros* package contains all of these capabilities, however, the focus was put on pure simulation and then pure control.

### Pure simulation of control with a modified GUI
- *mycobot320_moveit.launch* is the ROS launch file. This executes the MoveIT framework and starts Rviz. This file is capable of running many Rviz/MoveIT based scripts. The standalone launch file also features the ability to simulate simple path-planning by dragging the end-effector.

https://github.com/user-attachments/assets/569527dc-a3d8-4c0a-978a-49e1e8273eff


- *test_plan2.py* is a custom script designed to create a GUI to input cartesian and quaternion coordinates (base link reference), with w (4th dimension) being set in the algorithm. After inputting values, the end-effector will simulate movement to the desired location. In this example, w is 0.70711, with other values inputted as shown:

https://github.com/user-attachments/assets/ed366d9c-ff4f-4637-83da-f443453a2156


## Current Progress on Robot Aruco Tracking

*panini_masterfollower_1.py* is the main script for tracking the marker.
- it uses a script from a related Elephant Robotics community project but it is poorly applied after tracking showed to not be reliable. However, the robot does offer a feedback when the marker is presented, so this is provides a basis for *panini_masterfollower_2.py*

*panini_masterfollower_2.py* is a work in progress and derived from *panini_masterfollower_1.py*
- This is the main script being developed as of now:
	- Uses the camera calibration information from *camera-calibration.py*
	- Detects and gets the pose of the marker from *detect.py*  
- Current steps involve deriving the transformation between the camera's coordinate system and that of the end-effector. This allows the pose of the marker to be transformed into the end-effector coordinates for the manipulation part of the algorithm.
- Future steps involve actively "looking" at the Aruco Marker by moving the end-effector along the XYZ axes.


