#!/home/robot/.virtualenvs/hand-eye/bin/python
import rospy
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import cv2
import open3d as o3d

rgbImg = []
depthImg = []
cam_info = []

def rgbImageCallBack(data):
    global rgbImg
    rgbImg = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

def depthImageCallBack(data):
    global depthImg
    depthImg = np.frombuffer(data.data, dtype=np.int16).reshape(data.height, data.width)

def caminfoCallBack(info):
    global cam_info
    cam_info = info.data

rospy.init_node('detector', anonymous=True)
rospy.Subscriber("rgbImage", Image, rgbImageCallBack)
rospy.Subscriber("depthImage", Image, depthImageCallBack)
rospy.Subscriber("caminfo", numpy_msg(Floats), caminfoCallBack)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    try:
        cv2.imshow("subscribed rgb image", rgbImg)
        cv2.waitKey(1)
    except:
        pass
    rate.sleep()