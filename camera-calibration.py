#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import glob
#import modules.constants as constants


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp=np.zeros((7*7,3), np.float32)
objp[:,:2]=np.mgrid[0:7,0:7].T.reshape(-1,2)

objpoints=[]
imgpoints=[]

images=glob.glob('/home/paninib/catkin_ws/src/ROS_aruco_detection/aruco_detection/src/*.jpeg', recursive=True)
print(' good so far')

for image in images:
    print("true!")
    print(image)
    img=cv.imread(image)
    cv.imshow('img1',img)
    cv.waitKey(1000)
    gray=cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    

    ret,corners = cv.findChessboardCorners(gray, (7,7), None)
    
    cv.imshow('img2',img)
    cv.waitKey(1000)

    if ret==True:
        objpoints.append(objp)
        corners2=cv.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
        imgpoints.append(corners2)

        img=cv.drawChessboardCorners(img, (7,7), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

##Calibration

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (1920,1080), None, None)

print("\nCamera matrix: ",mtx)
print("\nDist coeff: ",dist)
print("\nRVEC: ",rvecs)
print("\nTVECS: ",tvecs)