#!/usr/bin/env python3

'''This will be master follower 2.
This should use detect.py to get the xy coordinates of the aruco marker. Z should be added later.
Then it should utilize all robotic math stuff from moveit package to get robot xy position.
It should map both xy sets and use some conditionals to adust the xy of the robot as the aruco xy moves.
The z is only if the xy works. the z should only be for zooming in purposes. 

First thing to do is clean up detect.py and do only 1 aruco marker (id=4)
Created 8/15/24
'''
import rospy, sys
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix,quaternion_from_matrix

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point, Quaternion


from pymycobot.mycobot import MyCobot
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


#from moveit_commander import RobotCommander, MoveGroupCommander


#some important numbers to use
    #orientation=np.array([-90, 36, -95])


#these are camera info from camera calibration: camera-calibration.py
cameramatrix= [[2.53434179e+03, 0.00000000e+00,  1.08436055e+03],
               [0.00000000e+00, 2.56676594e+03, 5.41210477e+02],
               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

distcoeffs=[ 0.20358547, -1.67815851,  0.0046789,  -0.0256404,   3.07001898]

mc=MyCobot('/dev/ttyUSB0',115200)
mc.send_angles([150, 30, 72, -19, -83, 13], 50)




class image_convert_pub:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback)

    #this is to get camera mat and dist coeffs through subscription, but better to use calibration data
    #self.camera_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_callback)

    self.cube_pose_pub = rospy.Publisher('/aruco_cube/pose', PoseStamped, queue_size=10)
    self.tf_broadcaster = tf.TransformBroadcaster()

    self.marker_pub = rospy.Publisher("/visualization_marker_real_cube", Marker, queue_size = 10)
    self.marker = Marker()

    self.camera_matrix= np.array(cameramatrix).reshape((3,3))
    self.dist_coeffs=np.array(distcoeffs)
   


  #This creates the pose arrows of aruco markers
  def create_marker(self, marker, marker_pub, type=2, target_location=None, color=None, scale=None):
      marker.header.frame_id = "camera_color_optical_frame"
      marker.header.stamp = rospy.Time.now()

      # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
      marker.type = type
      marker.id = 0

        # Set the scale of the marker
      marker.scale.x = scale
      marker.scale.y = scale
      marker.scale.z = scale

        # Set the color
      marker.color.r = color[0]
      marker.color.g = color[1]
      marker.color.b = color[2]
      marker.color.a = 1.0

        # Set the pose of the marker
      marker.pose.position = target_location.pose.position
      marker.pose.orientation = target_location.pose.orientation
      marker_pub.publish(marker)



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    markers_img, ids_list, corners = self.detect_aruco(cv_image)
    print(ids_list)
    print("corners:  ")
    print(corners)
    ids_list2=ids_list.flatten()
    print(ids_list2)



    if len(ids_list2)==1 and int(ids_list2[0])==4:
      for (markerCorner,markerID) in zip(corners, ids_list2):
        corners=markerCorner.reshape((4,2))
        (topLeft, topRight, bottomRight, bottomLeft)=corners
        print("corners again:  ")
        print(markerCorner)

      topRight=(int(topRight[0]), int(topRight[1]))
      topLeft=(int(topLeft[0]), int(topLeft[1]))
      bottomRight=(int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft=(int(bottomLeft[0]), int(bottomLeft[1]))

      print("the actual corners")
      print(topRight,topLeft,bottomRight,bottomLeft)

      cv2.line(cv_image, topLeft, topRight, (0,255,0),2)
      cv2.line(cv_image, topRight, bottomRight, (0,255,0),2)
      cv2.line(cv_image, bottomRight, bottomLeft, (0,255,0),2)
      cv2.line(cv_image, bottomLeft, topLeft, (0,255,0),2)

      #Finds the marker's center and labels it
      markercenter=np.mean(corners,axis=0)
      cv2.circle(cv_image, (int(markercenter[0]),int(markercenter[1])), 5, (0,255,255),-1)
      cv2.putText(cv_image, "Coordinates: "+str(markercenter), (200, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)

      if self.camera_matrix is not None and self.dist_coeffs is not None:
        
        ret = aruco.estimatePoseSingleMarkers(markerCorner, 0.06, self.camera_matrix, self.dist_coeffs)  # Assuming marker size is 0.05 meters

        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        print("RVEC and TVEC")
        print(ret)
        print(rvec)
        print(tvec)
        cv2.putText(cv_image, "RVEC: "+str(rvec), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
        cv2.putText(cv_image, "TVEC: "+str(tvec), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
        
        cv_image=cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)  # Draw axis with length 0.1 meter
        #cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids_list)

                    # Extract translation values from the pose
        translation = tvec.flatten() 
        translation[2] -= 0.1 # adjustment for basse_link to aruco_cube pose (TODO recaliberate)

            # Extract quaternion values from the pose
        rotation = rvec.ravel()
        quaternion = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        print("marker quat")
        print(quaternion)
        print("tvec with -0.1 z-adjustment")
        print(translation)
        
        mc.get_coords()
        #roboteulercoords=mc.get_coords()
        #print("this is the euler coords of robot")
        #print(roboteulercoords)
        #roboteulerorientation=roboteulercoords[-3:]
        #print(roboteulerorientation)
        #robotquat=quaternion_from_euler(roboteulercoords[:3])
        #print("this is the quat coords of robot")
        #robotquat2=quaternion_matrix(robotquat)
        #print(robotquat)

        cube_pose_msg = PoseStamped()
        cube_pose_msg.header.frame_id = 'camera_color_optical_frame'
        cube_pose_msg.header.stamp = rospy.Time.now()
        cube_pose_msg.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])
        cube_pose_msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        self.cube_pose_pub.publish(cube_pose_msg)

        
        print("full aruco pose")
        print(cube_pose_msg.pose.position)
        current_coords=mc.get_coords()
        print(current_coords)
        current_coords[2] -= 0.2
        #newpos.pose.position=euler_from_quaternion(Point(x=translation[0], y=translation[1], z=translation[2]))
        neworientation=np.array(euler_from_quaternion(quaternion))
        print(neworientation)

        newpos=np.array(translation)
        array1=np.concatenate((newpos,neworientation))

        mc.send_angles([50, 11.5, 73, -8, -83, 12.7],50)
        #mc.send_coords(array1,50)

        
        #this should do the math between two transforms

        #convert aruco quat to matrix
      

        #covert robot quat pose to matrix
        #self.reference_frame = "joint1"
        #self.arm.set_pose_reference_frame(self.reference_frame)
        '''
        roboteulercoords=np.array(mc.get_coords())
        print("this is the euler coords of robot")
        print(roboteulercoords)

        robotquat=np.array(quaternion_from_euler(roboteulercoords))
        print("this is the quat coords of robot")
        robotquat2=quaternion_matrix(robotquat)
        print(robotquat2)
        
        
        target_pose=PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position = Point(x=roboteulercoords[0], y=roboteulercoords[1], z=roboteulercoords[2])
        target_pose.pose.orientation = Quaternion(x=robotquat[0], y=robotquat[1], z=robotquat[2], w=robotquat[3])
        


        #matrix multiplication and get quat from matrix
        transform_worldmarkermatrix=np.dot(aruco_rotation_mat,robotquat2)
        worldtomarker_quat=quaternion_from_matrix(transform_worldmarkermatrix)
        print("this is new quats")
        print(worldtomarker_quat)

        '''

        self.create_marker(self.marker, self.marker_pub, type=1, target_location=cube_pose_msg, color=[1, 0, 0], scale=0.075) # create marker for rviz visualization
      
    aruco_rotation_mat=quaternion_matrix(quaternion)
    print("this is aruco rotation mat")
    print(aruco_rotation_mat)

    #roboteulercoords=mc.get_coords()
    #print("this is the euler coords of robot")
    #print(roboteulercoords)
    #roboteulerorientation=roboteulercoords[-3:]
    #print(roboteulerorientation)

      #robotquat=quaternion_from_euler(roboteulercoords[:3])
      #print("this is the quat coords of robot")
      #robotquat2=quaternion_matrix(robotquat)
      #print(robotquat2)

    

      


    cv2.imshow("Image", cv_image)
    key=cv2.waitKey(1) & 0xFF

    if ids_list is None:
      self.id_pub.publish(ids_list)
    else:
      ids_str = ''.join(str(e) for e in ids_list)
      self.id_pub.publish(ids_str)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(e)



  

  def detect_aruco(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the sruco markers and display its aruco id.
    return output, ids, corners

class cobot_tracks_aruco:
  
  def __init__(self):
    '''moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the ROS node,初始化ROS节点
    rospy.init_node("moveit_ik_demo")

    self.scene=moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)
    self.arm = moveit_commander.MoveGroupCommander("arm_group")

    # Get the name of the terminal link,获取终端link的名称
    self.end_effector_link = self.arm.get_end_effector_link()

    # Set the reference coordinate system used for the target position
        # 设置目标位置所使用的参考坐标系
    self.reference_frame = "joint1"
    self.arm.set_pose_reference_frame(self.reference_frame)

        # Allow replanning when motion planning fails,当运动规划失败后，允许重新规划
    self.arm.allow_replanning(True)

        # Set the allowable error of position (unit: meter) and attitude (unit: radian)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    self.arm.set_goal_position_tolerance(0.01)
    self.arm.set_goal_orientation_tolerance(0.05)'''
    print("initialized")



  def convert_rvec_to_quat(rvec):
    rot_mat, _ = cv2.Rodrigues(rvec)
    quat=tf.transformations.quaternion_from_matrix(np.vstack([rot_mat, np.array([0,0,0])]))
    return quat
  
  def moving(self):
    target_pose = PoseStamped()
    target_pose.header.frame_id = self.reference_frame
    target_pose.header.stamp = rospy.Time.now()
    '''target_pose.pose.position.x = -0.132
    target_pose.pose.position.y = -0.150
    target_pose.pose.position.z = 0.075
    target_pose.pose.orientation.x = 0.026
    target_pose.pose.orientation.y = 1.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.014'''

    orientation=quaternion_from_euler(3.1415, 0, -1.57)
    target_pose.pose.position.x = -0.132
    target_pose.pose.position.y = -0.150
    target_pose.pose.position.z = 0.075
    target_pose.pose.orientation.x = orientation[0]
    target_pose.pose.orientation.y = orientation[1]
    target_pose.pose.orientation.z = orientation[2]
    target_pose.pose.orientation.w = orientation[3]

    



    



  

def main():
  print("Initializing ROS-node")
  rospy.init_node('detect_markers', anonymous=True)

  #calls the aruco detection
  ic = image_convert_pub()


  #calls the robot movement
  #track=cobot_tracks_aruco()
  
  #continuous loop
  rospy.spin()

if __name__ == '__main__':
    main()