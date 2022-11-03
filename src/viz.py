#!/usr/bin/env python3
# importing libraries
import cv2
import requests
import numpy as np
#from PIL import Image
import time
from cv_bridge import CvBridge
import rospy
from micasense_wedge.msg import micasense
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt

kernel = np.ones((5, 5), np.float32)/25

# initializing
bridge = CvBridge()
rospy.init_node('viz')
pub = rospy.Publisher("micasense_viz", Image, queue_size=10)

# callback function
def callback(img):
	# 
	# Something funny is happening here and is causing the images to come out with heaps of noise.
	# I cant figure it out but I think it has to do with the conversion that is occuring from the 
	# message to the cv image.
	#
	# 5 channel array
	mxn = bridge.imgmsg_to_cv2(img.img_r).shape	# dimmensions of matrix
	w = mxn[1]		# width of array
	h = mxn[0]		# height of array
	#m = np.zeros((h, w*3, 3))	# creating 0 matrix to fill
	#m[0:h, 0:w, 0] += np.right_shift(bridge.imgmsg_to_cv2(img.img_r), 3)		# array for red channel
	#m[0:h, 0:w, 1] += np.right_shift(bridge.imgmsg_to_cv2(img.img_g), 3)		# array for green channel
	#m[0:h, 0:w, 2] += np.right_shift(bridge.imgmsg_to_cv2(img.img_b), 3)	# array for blue channel
	#m[0:h, (w):(w*2), 1] += np.right_shift(bridge.imgmsg_to_cv2(img.img_nir), 3)	# array for near IR
	#m[0:h, (w*2):(w*3), 2] += np.right_shift(bridge.imgmsg_to_cv2(img.img_swir), 3)	# array for swir
	r = np.right_shift(bridge.imgmsg_to_cv2(img.img_r), 9)		# array for red channel
	g = np.right_shift(bridge.imgmsg_to_cv2(img.img_g), 9)		# array for green channel
	b = np.right_shift(bridge.imgmsg_to_cv2(img.img_b), 9)	# array for blue channel
	n = np.right_shift(bridge.imgmsg_to_cv2(img.img_nir), 9)	# array for near IR
	s = np.right_shift(bridge.imgmsg_to_cv2(img.img_swir), 9)	# array for swir
	m = np.concatenate((r, g, b, n, s), axis=1).astype(np.uint8)
	#print(m.dtype)
	#cv2.imwrite('boing.jpg',m)		# saving images
	
	#plt.imshow(m)
	#plt.show()
	m = bridge.cv2_to_imgmsg(m.astype(np.uint8), encoding='passthrough')
	# publishing images
	pub.publish(m)
	print("pub")
# subsciber + initializer
rospy.Subscriber('micasense_data', micasense, callback)

rospy.spin()
