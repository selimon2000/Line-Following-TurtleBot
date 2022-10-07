#! /usr/bin/env python

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class LineFollower(object):

	def __init__(self):
		self.bridge_object=CvBridge()
		self.float_sub=rospy.Subscriber("/UltrasonicDistance",Float32,self.distance_callback)
		self.image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)
		self.cmd_vel_pub=rospy.Publisher("/cmd_vel",Twist, queue_size=10)
		self.distance=0;

	def camera_callback(self,data):
	
		twistObject=Twist()
		
		if self.distance==0:
			try:
				cv_image=self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
			except CvBridgeError as e:
				print(e)
				
			height, width, channels = cv_image.shape
			cv_image=cv_image[int(height/2):height, 0:width]

			frame=cv.GaussianBlur(cv_image, (7,7), cv.BORDER_DEFAULT)
			frame=cv.cvtColor(frame, cv.COLOR_BGR2HSV)

			
			#for colour of metallic yellow
			minThresh=np.array([20,155,205]) 	
			maxThresh=np.array([28,255,255])

			frame = cv.inRange(frame, minThresh, maxThresh)

			#FINDING CENTROID OF BINARY BLOB by calculating moments of binary image
			M = cv.moments(frame)
			# calculate x,y coordinate of center
			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			else:
				cX, cY = 0, 0
			#highlight the center
			cv.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
			print(cX)
			print(cY)		    
			cv.imshow("Image Window", cv_image)
			
			deltaX=cX-width/2
			deltaZ=-deltaX/900

			twistObject.angular.z=deltaZ
			twistObject.linear.x=0.07
			print("Distance from midpoint to blob: "+str(deltaX))
			print("Rotating speed of z axis: "+str(deltaZ))

			cv.waitKey(1)
			
		elif self.distance==1:
			twistObject.angular.z=0
			twistObject.linear.x=0	
				
		self.cmd_vel_pub.publish(twistObject)


	def distance_callback(self,data):
		print("Distance (using Ultrasonic Sensor): "+ str(data.data))
		
		if data.data<20:
			self.distance=1
			print("STOPPING!")
		else:
			self.distance=0			

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
