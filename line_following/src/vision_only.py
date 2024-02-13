#! /usr/bin/env python

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class LineFollower(object):

	def __init__(self):
		self.bridge_object=CvBridge()
		self.image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)

	def camera_callback(self,data):

		try:
			cv_image=self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)
		
		height, width, channels = cv_image.shape
		cv_image=cv_image[int(height/2):height, 0:width]

		frame=cv.GaussianBlur(cv_image, (7,7), cv.BORDER_DEFAULT)
		frame=cv.cvtColor(frame, cv.COLOR_BGR2HSV)

		#for colour of blue
		#minThresh=np.array([101,50,38])
		#maxThresh=np.array([130,255,255])
		
		#for colour of metallic yellow
		minThresh=np.array([21,166,217]) 
		maxThresh=np.array([27,255,255])

		frame = cv.inRange(frame, minThresh, maxThresh)

		#FINDING CENTROID OF BINARY BLOB by calculating moments of binary image
		M = cv.moments(frame)
		# calculate x,y coordinate of center
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
		else:
			cX, cY = 0, 0
				    
		# put text and highlight the center
		cv.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
		print(cX)
		print(cY)		    
		cv.imshow("Image Window", cv_image)
		
		deltaX=cX-width/2
		twistObject=Twist()
		twistObject.angular.z=-deltaX/100
		twistObject.linear.x=0.2
		print("Angular Value sent: " + str(deltaX))
		
		cv.waitKey(1)  

def main():
	line_follower_object=LineFollower()
	rospy.init_node('line_following_node', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv.destroyAllWindows()

if __name__ == '__main__':
	main()
