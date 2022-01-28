#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import  Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.msg import Odometry


def RecoverCLAHE(sceneRadiance):
    #clahe = cv2.createCLAHE(clipLimit=2, tileGridSize=(8, 8))
    #sceneRadiance[:, :, 0] = clahe.apply((sceneRadiance[:, :, 0]))
    #sceneRadiance[:, :, 1] = clahe.apply((sceneRadiance[:, :, 1]))
    #sceneRadiance[:, :, 2] = clahe.apply((sceneRadiance[:, :, 2]))
    sceneRadiance[:, :, 0] =  cv2.equalizeHist(sceneRadiance[:, :, 0])
    sceneRadiance[:, :, 1] =  cv2.equalizeHist(sceneRadiance[:, :, 1])
    sceneRadiance[:, :, 2] =  cv2.equalizeHist(sceneRadiance[:, :, 2])

    return sceneRadiance
class image_converter:

    def __init__(self):
        self.is_sim = rospy.get_param("~sim")
        

         
        self.enhance_topic = rospy.get_param("~enhance_topic", "/sam/detection/image_enhance_down")

        self.cam_image = rospy.get_param("~cam_image", "/sam/perception/camera_down/image_color")
        self.cam_info = rospy.get_param("~cam_info", "/sam/perception/camera_down/camera_info")
        

        self.dr_local_odom = rospy.get_param("~dr_local_odom", "/sam/dr/local/odom/filtered")

        
        self.image_enhance_pub = rospy.Publisher(self.enhance_topic, Image, queue_size=1)
        

        self.bridge = CvBridge()
        if self.is_sim:
            rospy.Subscriber(self.cam_image, Image, self.callback, queue_size=1, buff_size=655360)
        else:
            rospy.Subscriber(self.cam_image, CompressedImage, self.callback, queue_size=1, buff_size=655360)

        self.cam_info_sub = rospy.Subscriber(self.cam_info, CameraInfo, self.pub_cam_info)
        rospy.Subscriber(self.dr_local_odom, Odometry, self.pub_sam_pos)
    
    def pub_cam_info(self, data):
        self.cam_info_K = np.array(data.K).reshape((3,3))
        self.cam_info_R = np.array(data.R).reshape((3,3))
        self.cam_info_P = np.array(data.P).reshape((3,4))

        self.cam_info_height = data.height
        self.cam_info_width = data.width
        
        self.cam_info_sub.unregister()


    def pub_sam_pos(self, data):
        self.sam_pos = data.pose.pose.position
        



  

    def callback(self,data):
        
            
        try:
            if self.is_sim:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            else:
                cv_image = np.fromstring(data.data, np.uint8)
                cv_image = cv2.imdecode(np.fromstring(cv_image, dtype=np.uint8), -1)
                
        except CvBridgeError as e:
            print(e)

        # image enhancement
        #cv_image = cv2.resize(cv_image, (640,360))
        cv_image = RecoverCLAHE(cv_image)
        
       
        try:
            self.image_enhance_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            
            rospy.sleep(0.5)
        except CvBridgeError as e:    
            print(e)
        

def main():
    rospy.init_node('image_converter', anonymous=True)
    
    ic_d = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

