#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import tf.transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf

from pymycobot.mycobot import MyCobot

#these are camera info from camera calibration: camera-calibration.py
cameramatrix= [[2.53434179e+03, 0.00000000e+00,  1.08436055e+03],
               [0.00000000e+00, 2.56676594e+03, 5.41210477e+02],
               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

distcoeffs=[ 0.20358547, -1.67815851,  0.0046789,  -0.0256404,   3.07001898]

mc=MyCobot('/dev/ttyUSB0',115200)

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


  '''
  # Extract camera matrix and distortion coefficients from CameraInfo message
  def camera_info_callback(self, msg):
    self.camera_matrix= np.array(cameramatrix).reshape((3,3))
    self.dist_coeffs=np.array(distcoeffs)

    print("camera mat and distortion coeffs")
    print(self.camera_matrix)
    print(self.dist_coeffs)
'''


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    markers_img, ids_list, corners = self.detect_aruco(cv_image)
    #print(ids_list)
    #print("corners:  ")
    #print(corners)
    ids_list2=ids_list.flatten()
    #print(ids_list2)



    if len(ids_list2)==1 and int(ids_list2[0])==4:
      for (markerCorner,markerID) in zip(corners, ids_list2):
        corners=markerCorner.reshape((4,2))
        (topLeft, topRight, bottomRight, bottomLeft)=corners
        #print("corners again:  ")
        #print(markerCorner)

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
        #print(ret)
        print(rvec)
        print(tvec)
        cv2.putText(cv_image, "RVEC: "+str(rvec), (10, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
        cv2.putText(cv_image, "TVEC: "+str(tvec), (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)
        
        cv_image=cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)  # Draw axis with length 0.1 meter
        #cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids_list)

                    # Extract translation values from the pose
        translation = tvec.flatten() 
        translation[2] -= 0.1 # adjustment for basse_link to aruco_cube pose (TODO recaliberate)

            # Extract quaternion values from the pose
        rotation = rvec.ravel()
        quaternion = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])

        cube_pose_msg = PoseStamped()
        cube_pose_msg.header.frame_id = 'camera_color_optical_frame'
        cube_pose_msg.header.stamp = rospy.Time.now()
        cube_pose_msg.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])
        cube_pose_msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        self.cube_pose_pub.publish(cube_pose_msg)
        print("this is aruco pose")
        print(cube_pose_msg)
        #print("robot coods: ")
        #print(mc.get_coords())

        aruco_rotation_mat=tf.transformations.quaternion_matrix(quaternion)
        print("this is aruco rotation mat")
        print(aruco_rotation_mat)

        roboteulercoords=np.array(mc.get_coords())
        print("this is the euler coords of robot")
        print(roboteulercoords)
        #roboteulerorientation=roboteulercoords[-3:]
        #print(roboteulerorientation)

        roboteulercoords[3]/=1000
        roboteulercoords[4]/=1000
        roboteulercoords[5]/=1000
 
        robotquat=tf.transformations.quaternion_from_euler(roboteulercoords[3], roboteulercoords[4], roboteulercoords[5])
        #*np.pi/180
        print("this is the quat coords of robot")
        print(robotquat)
        robotquat2=tf.transformations.quaternion_matrix(robotquat)
        print("put into matrix")
        print(robotquat2)

        #matrix multiplication and get quat from matrix
        transform_worldmarkermatrix=np.dot(aruco_rotation_mat,robotquat2)
        print("this is multiplied matrices")
        print(transform_worldmarkermatrix)
        worldtomarker_quat=tf.transformations.quaternion_from_matrix(transform_worldmarkermatrix)
        print("this is new quats")
        print(worldtomarker_quat)

        newrobotorientation=tf.transformations.euler_from_quaternion(worldtomarker_quat)
        print("new robot orientation with euler")
        newrobotorientation=np.array(newrobotorientation)
        print(newrobotorientation)
        newrobotorientation[0] =newrobotorientation[0]*10
        newrobotorientation[1] =newrobotorientation[1]*10
        newrobotorientation[2] =newrobotorientation[2]*10
        tosendarray=[roboteulercoords[0],roboteulercoords[1],roboteulercoords[2],
          newrobotorientation[0], newrobotorientation[1], newrobotorientation[2]]
        print(tosendarray)
        mc.send_coords[[tosendarray[0],tosendarray[1],tosendarray[2],tosendarray[3],tosendarray[4],tosendarray[5]],50]
        

        self.create_marker(self.marker, self.marker_pub, type=1, target_location=cube_pose_msg, color=[1, 0, 0], scale=0.075) # create marker for rviz visualization


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



def main():  
  print("Initializing ROS-node")
  rospy.init_node('detect_markers', anonymous=True)

  mc.send_angles([49.48, -5.18, 90.79, 0.87, -90.26, 7.73], 50)


  print("Bring the aruco-ID in front of camera")
  ic = image_convert_pub()
  rospy.spin()

if __name__ == '__main__':
    main()