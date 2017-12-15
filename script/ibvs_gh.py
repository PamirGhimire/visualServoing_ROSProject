#!/usr/bin/env python

# DESCRIPTION:
# i/o:
# This node subscribes to rgb_image topic of turtlebot's kinect
# and publishes command velocities to /ibvs_gh/cmd_vel

# task:
# detect qr tag in the rgb image
# extract position of corners of tag in the image
# compute velocities to move detected corners to desired position
# transform velocities from cam frame to robot's frame

# Pamir Ghimire, 10 December, 2017
# resources:
# 1. convert ros images to cv::mat
# http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
#
# Blanchon Marc, 15 december, 2017
# Integration


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge as bridge

import numpy as np
import cv2

import matlab_adaptation as Keypoints

global Keypoints_x_curr , Keypoints_y_curr

#---------------------------------------------
# Listener : Listens to rgb image stream
#---------------------------------------------
def imlistener(Keypoints_x , Keypoints_y, Keypoints_x_curr , Keypoints_y_curr):
	rospy.init_node('listener_ibvs_gh', anonymous = False)

	# subscriber : topic name, message type, callback_fun
	rospy.Subscriber("/camera/rgb/image_raw", Image, callback_ibvs)

	estimation_x , estimation_y = Keypoints.Estimate(Keypoints_x , Keypoints_y ,  Keypoints_x_curr , Keypoints_y_curr )

	rospy.spin()

	return


#---------------------------------------------
# Listener's callback : processes images, outputs ibvs_velocity
#---------------------------------------------
def callback_ibvs(msg):
	# convert sensor_msgs/Image to cv::mat
    imgbrg = bridge()
    img = bridge.imgmsg_to_cv2(imgbrg, msg, desired_encoding="passthrough")

    # convert rgb image to grayscale
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY,dstCn=0)

	Keypoints_x_curr , Keypoints_y_curr = Keypoints.__main__(False,img);

    # test writing image (works!)
    # cv2.imwrite('somefile.png', img)

if __name__ == '__main__':
   try:
	   Keypoints_x_curr = np.array([])
	   Keypoints_y_curr = np.array([])
	  Keypoints_x , Keypoints_y = Keypoints.__main__(True);
      imlistener(Keypoints_x , Keypoints_y,  Keypoints_x_curr , Keypoints_y_curr)
   except rospy.ROSInterruptException:
      pass
