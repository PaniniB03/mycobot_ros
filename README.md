# mycobot_ros
***This repo is a work in progress! The files are individually imported from Linux, not an accurate representation of the file hierarchy.
This will be updated once the final tracking program is finished.***

Cloned from Elephant Robotic's *mycobot_ros* repo 

## Aruco Marker Detection

- *camera-calibration.py* is to calibrate a Logitech Brio Webcam on the end-effector.
	- All jpeg images were taken with the webcam and used for calibration purposes.
	- This returns a camera matrix, distortion coefficients, Rodrigues Vector (rvec), and translation vector (tvec).

- *usb_cam-stream.launch* is the ROS launch file to initialize ROS nodes for the webcam to operate.
- *detect.py* is the script to actively detect the Aruco Marker's 6-DOF Pose, relative to webcam. It is also easily modifiable to detect multiple markers if needed.
	- Demonstration: 


## Robot Control & Simulation

There are several ways to simulate it via Rviz, control the robot without Rviz, and control the robot with Rviz simulating real-time movement. The *mycobot_ros* package contains all of these capabilities, however, the focus was put on pure simulation and then pure control.

### Pure simulation of control with a modified GUI
- *mycobot320_moveit.launch* is the ROS launch file. This executes the MoveIT framework and starts Rviz. This file is capable of running many Rviz/MoveIT based scripts. The standalone launch file also features the ability to simulate simple path-planning by dragging the end-effector.

https://github.com/user-attachments/assets/569527dc-a3d8-4c0a-978a-49e1e8273eff


- *test_plan2.py* is a custom script designed to create a GUI to input cartesian and quaternion coordinates (base link reference), with w (4th dimension) being set in the algorithm. After inputting values, the end-effector will simulate movement to the desired location. In this example, w is 0.70711, with other values inputted as shown:

https://github.com/user-attachments/assets/ed366d9c-ff4f-4637-83da-f443453a2156


## Robot Aruco Tracking Progress

*panini_masterfollower_1.py* is the main script for tracking the marker.
- it uses a script from a related Elephant Robotics' community project but it is poorly applied since tracking was proven to not be reliable. However, the robot offers  feedback when the marker is presented, so this is some progress and provides so basis for *panini_masterfollower_2.py*


*panini_masterfollower_2.py* is a work in progress and derived from *panini_masterfollower_1.py*
- this is the main script being developed.
- Current steps involve using kinematic transformations to accurately "look" at the Aruco Marker


