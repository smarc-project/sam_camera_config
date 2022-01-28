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
        
        self.car_depth = rospy.get_param("~car_depth")
        self.threshold = rospy.get_param("~threshold", "/sam/detection/threshold") 

        self.mask_topic = rospy.get_param("~mask_topic", "/sam/detection/image_mask_down")  
        self.enhance_topic = rospy.get_param("~enhance_topic", "/sam/detection/image_enhance_down")
        self.poi_topic = rospy.get_param("~poi_topic", "/sam/detection/poi_down")

        self.cam_image = rospy.get_param("~cam_image", "/sam/perception/camera_down/image_color")
        self.cam_info = rospy.get_param("~cam_info", "/sam/perception/camera_down/camera_info")
        self.cam_frame = rospy.get_param("~cam_frame", "sam/camera_down_link")

        self.dr_local_odom = rospy.get_param("~dr_local_odom", "/sam/dr/local/odom/filtered")

        self.mask_pub = rospy.Publisher(self.mask_topic, Image, queue_size=1)
        self.image_enhance_pub = rospy.Publisher(self.enhance_topic, Image, queue_size=1)
        self.position_pub = rospy.Publisher(self.poi_topic, PointStamped, queue_size=1)

        self.cnt = 0
        self.backSub = cv2.createBackgroundSubtractorKNN()

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
        # print(self.cam_info_height)
        self.cam_info_sub.unregister()


    def pub_sam_pos(self, data):
        self.sam_pos = data.pose.pose.position
        

    
    def pos_estimation(self,bounding_box):

        d = 1.5 # the diagnols of the bouding box shoud be 4.0857 m, calculated using Mini's length and width  
        # since we can only see the car from a very close distance, we are not going to see the whole car the whole time
        P_00 = self.cam_info_P[0][0]

        rel_pos_list = [] 
        for x,y,w,h in bounding_box:
            z_ = (P_00*d)/np.sqrt(w**2+h**2)
            p = z_*np.transpose(np.array([(x+w/2), (y+h/2), 1]))
            # p0 = z_*np.transpose(np.array([680, 512, 1]))
            
            X3 = np.dot(np.linalg.pinv(self.cam_info_P), p) # u=x+w/2, v=y+h/2
            # X0 = np.dot(np.linalg.pinv(self.cam_info_P), p0) # u=self.cam_info_width/2, v=self.cam_info_height/2

            rel_pos_list.append(X3)
            
        return rel_pos_list

    def filter_mask(self, img):

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))

        # Fill any small holes
        closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        # Remove noise
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)

        # Dilate to merge adjacent blobs
        dilation = cv2.dilate(opening, kernel, iterations=2)

        # threshold
        th = dilation[dilation < 240] = 0

        return th, dilation

    def find_car(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        bounding_box = []
        h_max, w_max  = img.shape[0], img.shape[1]

        diff = self.backSub.apply(gray)
        th, dia = self.filter_mask(diff)
        cnts = cv2.findContours(
            dia, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            if float(h)/h_max>0.05 or float(w)/w_max>0.05:

                bounding_box.append([x,y,w,h])

                dia  = cv2.rectangle(dia, (x, y), (x + w, y + h), (127,0,255), 2)

        return dia, bounding_box


    def find_yellow(self, img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower = np.array([20, 100, 100], dtype="uint8")
        upper = np.array([75, 255, 255], dtype="uint8")

        mask_yellow = cv2.inRange(hsv,lower,upper)
        yellow_ratio =(cv2.countNonZero(mask_yellow))/(img.size/3.)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 0)
        h_max, w_max  = img.shape[0], img.shape[1]
        bounding_box = []
        cnts = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            # print(x,y,w,h, float(h)/h_max)
            if float(h)/h_max>0.05 or float(w)/w_max>0.05:
                bounding_box.append([x,y,w,h])
                mask_yellow = cv2.rectangle(mask_yellow, (x, y), (x + w, y + h), (127,0,255), 2)
        return yellow_ratio, mask_yellow, bounding_box

    def callback(self,data):
        #self.cnt+=1
        #if self.cnt==5:
            
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
        
        # yellow_ratio, mask_yellow, bounding_box = self.find_yellow(cv_image)
        #dia, bounding_box = self.find_car(cv_image)
        #cv_image[:, :, 0] =  cv2.equalizeHist(cv_image[:, :, 0])
        #cv_image[:, :, 1] =  cv2.equalizeHist(cv_image[:, :, 1])
        #cv_image[:, :, 2] =  cv2.equalizeHist(cv_image[:, :, 2])
        # print(yellow_ratio, self.threshold)
        # if yellow_ratio < self.threshold:

            # rospy.loginfo_throttle(5, "Considered no car detected, yellow_ratio is {0:0.6f} but threshold is {1:0.6f}".format(yellow_ratio, self.threshold))

        # else:
        #if len(bounding_box)>0:
               
            #rel_pos_list = self.pos_estimation(bounding_box)
            #if  len(rel_pos_list)>0:
                #for rel_pos_ in rel_pos_list:
                    # print("rel_pos: ", rel_pos_)
                    # pos = Point()
                    #pos = PointStamped()
                    #pos.header.frame_id = self.cam_frame
                
                    # wrt to the camera frame
                    #pos.point.x = rel_pos_[0] 
                    #pos.point.y = rel_pos_[1] 
                    #pos.point.z = rel_pos_[2] 
                    # print("depth,  depth-sam.pos.z: ", self.car_depth, self.car_depth - self.sam_pos.z )
                    # print(pos.point.z)
                    # if the detected objected is at roughly the same depth as we think the car is then publish the rel_pos in the camera_frame
                    #if abs(self.sam_pos.z - pos.point.z +self.car_depth)<3:
                    #    self.position_pub.publish(pos)
                    #else:
                    #    rospy.loginfo_throttle(5, "Distance between car and SAM estimated from detection is {0:.2f} but expected to be {1:.2f}".format(pos.point.z, (self.car_depth+self.sam_pos.z)))
        #r = rospy.Rate(2) # 10hz
        #while not rospy.is_shutdown():
        #    pub.publish("hello")
        #    r.sleep()
        try:
            self.image_enhance_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #self.mask_pub.publish(self.bridge.cv2_to_imgmsg(dia, "mono8"))
            rospy.sleep(0.5)
        except CvBridgeError as e:    
            print(e)
        #self.cnt=0

def main():
    rospy.init_node('image_converter', anonymous=True)
    
    ic_d = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

