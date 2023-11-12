import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose2D

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,

}

class ArUcoLocalisationNode(Node):
    def __init__(self):
        super().__init__('aruco_localisation_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  
            self.image_callback,
            10
        )
        self.pose_publisher = self.create_publisher(Pose2D, '/detected_aruco', 10)
        self.cv_image = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)  # ill check and replace with dictionary ussed in eyrc
        #mostly 6x6 hai
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.marker_size = 0.1  ## ill check and replace with size used in eyrc
        self.camera_matrix = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 1]])  
        self.distortion_coefficients = np.array([0, 0, 0, 0, 0])  



    
    # def aruco_display(corners, ids, rejected, image):
    
    #     if len(corners) > 0:
            
    #         ids = ids.flatten()
            
    #         for (markerCorner, markerID) in zip(corners, ids):
                
    #             corners = markerCorner.reshape((4, 2))
    #             (topLeft, topRight, bottomRight, bottomLeft) = corners
                
    #             topRight = (int(topRight[0]), int(topRight[1]))
    #             bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    #             bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    #             topLeft = (int(topLeft[0]), int(topLeft[1]))

    #             cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
    #             cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
    #             cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
    #             cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
    #             cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    #             cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    #             cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
    #             cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
    #                 0.5, (0, 255, 0), 2)
    #             print("[Inference] ArUco marker ID: {}".format(markerID))			
    #     return image 

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting Image message: %s' % str(e))
            return

        if self.cv_image is not None:
          
            corners, ids, _ = cv2.aruco.detectMarkers(self.cv_image, self.aruco_dict, parameters=self.aruco_parameters)

            if ids is not None:
                for i in range(len(ids)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.camera_matrix, self.distortion_coefficients)
                    print ("the transVec: ", tvec )
                    print ("the RotVec: ", rvec )
                    x = (tvec[0][0][0])
                    y = (tvec[0][0][1])
                    theta = (rvec[0][0][2])  
                    # Assuming rvec is in radians
                    # Convert theta from radians to degrees if needed
                    
                    pose_msg = Pose2D()
                    pose_msg.x = x
                    pose_msg.y = y
                    pose_msg.theta = theta
                    self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_localisation_node = ArUcoLocalisationNode()
    print("Rospy spin hua")
    rclpy.spin(aruco_localisation_node)
    print("Rospy spin khatam")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
