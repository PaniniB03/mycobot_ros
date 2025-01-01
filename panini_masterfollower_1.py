#!/usr/bin/env python3

from typing import Tuple

import cv2
import cv2.aruco as aruco

import numpy as np
import tf

import rospy
from pymycobot.mycobot import MyCobot

import time
import platform

'''from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Point, Quaternion'''

mc=MyCobot('/dev/ttyUSB0',115200)

#calculate target position based on camera coordinates
def model_track(camera):
    model_pos = np.array([-camera[0], -camera[2], -camera[1]])
    camera_pos = np.array([-37.5, 416.6, 322.9])
    target_pos = model_pos + camera_pos
    print("model_pos", model_pos)
    # print("target_pos", target_pos)
    return target_pos

'''
def calculate_similarity(camera):
    n = camera.shape[0]
    total_similarity = 0
    for i in range(n):
        for j in range(i+1, n):
            vector_a = camera[i]
            vector_b = camera[j]
            dot_product = np.dot(vector_a, vector_b)
            norm_a = np.linalg.norm(vector_a)
            norm_b = np.linalg.norm(vector_b)
            similarity = dot_product / (norm_a * norm_b)
            total_similarity += similarity
    return total_similarity/n

def similarity_change_rate(new_similarity):
    global prev_similarity
    if prev_similarity is None:
        prev_similarity = new_similarity
        return 0
    else:
        change_rate = (new_similarity - prev_similarity) / prev_similarity
        prev_similarity = new_similarity
        return change_rate

'''

#this function used to convert rotation matrix to euler angles
def CvtRotationMatrixToEulerAngle(pdtRotationMatrix):
    pdtEulerAngle = np.zeros(3)

    pdtEulerAngle[2] = np.arctan2(pdtRotationMatrix[1, 0], pdtRotationMatrix[0, 0])

    fCosRoll = np.cos(pdtEulerAngle[2])
    fSinRoll = np.sin(pdtEulerAngle[2])

    pdtEulerAngle[1] = np.arctan2(-pdtRotationMatrix[2, 0], (fCosRoll * pdtRotationMatrix[0, 0]) + (fSinRoll * pdtRotationMatrix[1, 0]))
    pdtEulerAngle[0] = np.arctan2((fSinRoll * pdtRotationMatrix[0, 2]) - (fCosRoll * pdtRotationMatrix[1, 2]), (-fSinRoll * pdtRotationMatrix[0, 1]) + (fCosRoll * pdtRotationMatrix[1, 1]))

    return pdtEulerAngle

#function used to convert euler to rotation mat
def CvtEulerAngleToRotationMatrix(ptrEulerAngle):
    ptrSinAngle = np.sin(ptrEulerAngle)
    ptrCosAngle = np.cos(ptrEulerAngle)

    ptrRotationMatrix = np.zeros((3, 3))
    ptrRotationMatrix[0, 0] = ptrCosAngle[2] * ptrCosAngle[1]
    ptrRotationMatrix[0, 1] = ptrCosAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] - ptrSinAngle[2] * ptrCosAngle[0]
    ptrRotationMatrix[0, 2] = ptrCosAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] + ptrSinAngle[2] * ptrSinAngle[0]
    ptrRotationMatrix[1, 0] = ptrSinAngle[2] * ptrCosAngle[1]
    ptrRotationMatrix[1, 1] = ptrSinAngle[2] * ptrSinAngle[1] * ptrSinAngle[0] + ptrCosAngle[2] * ptrCosAngle[0]
    ptrRotationMatrix[1, 2] = ptrSinAngle[2] * ptrSinAngle[1] * ptrCosAngle[0] - ptrCosAngle[2] * ptrSinAngle[0]
    ptrRotationMatrix[2, 0] = -ptrSinAngle[1]
    ptrRotationMatrix[2, 1] = ptrCosAngle[1] * ptrSinAngle[0]
    ptrRotationMatrix[2, 2] = ptrCosAngle[1] * ptrCosAngle[0]

    return ptrRotationMatrix

#function fo visual tracking and calculating target position
def Visual_tracking(coord, camera):
    pose_camera = camera[:3]
    angle_camear = camera[3:]
    r = CvtEulerAngleToRotationMatrix(angle_camear)
    euler = coord[3:]
    R = CvtEulerAngleToRotationMatrix(euler)
    offset = np.array([0, -33.9, -138])
    Roff = np.array([[1, 0, 0],
                     [0, -1, 0],
                     [0, 0, -1]])
    vector = pose_camera + offset
    pos = coord[:3] + np.dot(np.dot(R, r), Roff).dot(vector)
    angle = np.array(CvtRotationMatrixToEulerAngle(np.dot(np.dot(R, r), Roff))) * 180/np.pi
    target = np.concatenate((pos, angle))
    return target

def Visual_tracking280(coord, camera):
    pose_camera = camera[:3]
    angle_camear = camera[3:]
    r = CvtEulerAngleToRotationMatrix(angle_camear)
    # r = np.array([[1, 0, 0],
    #                  [0, 1, 0],
    #                  [0, 0, 1]])
    euler = np.radians(coord[3:])
    R = CvtEulerAngleToRotationMatrix(euler)
    offset = np.array([0, 0, -250])
    Roff = np.array([[1, 0, 0],
                     [0, -1, 0],
                     [0, 0, -1]])
    # Roff = np.array([[1, 0, 0],
    #                  [0, 1, 0],
    #                  [0, 0, 1]])
    vector = pose_camera + offset
    # print("R", R)
    # print("r", r)

    move_pos = np.dot(np.dot(R, r), Roff).dot(vector)
    pos = coord[:3] + move_pos
    # angle = np.array(CvtRotationMatrixToEulerAngle(np.dot(np.dot(R, r), Roff))) * 180/np.pi
    angle =  coord[3:]
    target = np.concatenate((pos, angle))
    return target

class Detect_marker():
    def __init__(self):

        # Creating a Camera Objectt
        if platform.system() == "Windows":
            cap_num = 0
            self.cap = cv2.VideoCapture(1)
            self.cap.set(3, 640)
            self.cap.set(4, 640)
        elif platform.system() == "Linux":
            cap_num = 4
            self.cap = cv2.VideoCapture(cap_num)
            self.cap.set(3, 640)
            self.cap.set(4, 640)
        # self.cap = cv2.VideoCapture('detect.mp4')

        # choose place to set cube
        self.color = 0

        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)


        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # 摄像头的内参矩阵
        self.camera_matrix = np.array([[2.53434179e+03, 0.00000000e+00,  1.08436055e+03],
               [0.00000000e+00, 2.56676594e+03, 5.41210477e+02],
               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        # 摄像头的畸变系数
        self.dist_coeffs = np.array(([ 0.20358547, -1.67815851,  0.0046789,  -0.0256404,   3.07001898]))

        # self.robot.init_robot()



#capture the target
    def show_video_v2(self):
        # self.robot.init_robot()
        xyz = np.array([0,0,0])
        LIST = []
        num_count = 0
        list_len = 5
        # cmax = [180, 80, 240]
        # cmin = [130, -80, 200]
        cmax = [150, -150, 300]
        cmin = [-150, -250, 200]

        while cv2.waitKey(1) < 0:
            success, img = self.cap.read()
            if not success:
                print("It seems that the image cannot be acquired correctly.")
                break
            # transfrom the img to model of gray
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Detect ArUco marker.
            corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if len(corners) > 0:
                if ids is not None:
# get informations of aruco
                    ret = cv2.aruco.estimatePoseSingleMarkers(
                        # '''https://stackoverflow.com/questions/53303730/what-is-the-value-for-markerlength-in-aruco-estimateposesinglemarkers'''
                        corners, 0.025, self.camera_matrix, self.dist_coeffs
                    )
# rvec:rotation offset,tvec:translation deviator
                    (rvec, tvec) = (ret[0], ret[1])
                    
                    (rvec - tvec).any()
                    xyz = tvec[0, 0, :] * 1000
                    rpy = rvec[0,0,:]

                    camera = np.array([xyz[0], xyz[1], xyz[2]])

                    if num_count > list_len:
                        target = model_track(camera)
                        print("target", target)

                        for i in range(3):
                            if target[i] > cmax[i]:
                                target[i] = cmax[i]
                            if target[i] < cmin[i]:
                                target[i] = cmin[i]

                        pose = np.array([-90, 36, -95])
                        coord = np.concatenate((target.copy(), pose), axis=0)

                        # q1 = math.atan(xyz[0] / xyz[2])*180/np.pi
                        mc.send_coords(coord,50,0)
                        
                        
                        # print('target', coord)
                        num_count = 1;
                    else:
                        num_count = num_count + 1
                    

                    for i in range(rvec.shape[0]):
                        # draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)
            cv2.imshow("show_video", img)

        return  xyz[0],xyz[1]

if __name__ == "__main__":
    if mc.get_fresh_mode() == 0:
        mc.set_free_mode(1)
    time.sleep(1)
    mc.set_tool_reference([0, 0, 130, 0, 0, 0])
    mc.set_end_type(1)
    # angles = mc.get_angles()
    # while len(angles) == 0:
    #     angles = mc.get_angles()
    # print("angles", angles)
    # exit()


    prev_similarity = None
    time.sleep(1)
    mc.send_angles([97, 0, 88, -3, 0, -33], 50)
    time.sleep(3)

    detect = Detect_marker()
    #mc.set_joint_max
    print('yes')
    detect.show_video_v2()






