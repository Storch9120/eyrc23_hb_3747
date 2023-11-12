#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
# from rclpy.logging
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np 
from geometry_msgs.msg import Pose2D

# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.aruco_rec = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # Subscribe the topic /camera/image_raw
        self.aruco_send = self.create_publisher(
            Pose2D, '/detected_aruco', 10
        )
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        # self.aruco_dict =  cv2.aruco.Dictionary_get(cv2.aruco.DICT_6x6_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # ill check and replace with dictionary ussed in eyrc
        
        self.posepub = Pose2D()

    def image_callback(self, msg):
        try:
            self.cv_image=self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except:
            self.get_logger().error('Conversion Error0')
            return
        
        if self.cv_image is not None:
            
            corners, ids, _ = cv2.aruco.detectMarkers(self.cv_image, self.aruco_dict, parameters=self.aruco_parameters)

            
            if len(corners) > 0:
                ids = ids.flatten()
                print("In corners loop")
                for (markerCorner, markerID) in zip(corners, ids):
                    
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    cv2.line(self.cv_image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(self.cv_image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(self.cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(self.cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                    
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    # cv2.circle(self.cv_image, (cX, cY), 4, (0, 0, 255), -1)
                    theta_calib_pt = [int(topLeft[0]+topRight[0]/2.0), int(topLeft[1]+topRight[1]/2.0)]  
                    slope = (theta_calib_pt[1]-cY)/(theta_calib_pt[0]-cX)
                    theta = np.arctan(slope)
                    
                    cv2.putText(self.cv_image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    # print("[Inference] ArUco marker ID: {}".format(markerID))

                #now convert corners to pose 2d after identifying id.
                    if (markerID == 1):
                        self.posepub.x=float(cX)
                        self.posepub.y=float(cY)
                        self.posepub.theta= float(theta) 
                        self.aruco_send.publish(self.posepub)
                        print("Pub hogaya, theta: "+ str(theta))

            cv2.imshow("test", self.cv_image)
            cv2.waitKey(1)
            # cv2.waitKey(0)
                
        #convert ROS image to opencv image
        #Detect Aruco marker
        #Publish the bot coordinates to the topic  /detected_aruco

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
