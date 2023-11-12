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


# Team ID:		[ Team-3747 ]
# Author List:		[ Furquan Shaikh, Satvik Sharma]
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench, Pose2D
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal   
from cv_bridge import CvBridge    
from rclpy.impl import rcutils_logger   
import cv2 
PI =3.14   
logger = rcutils_logger.RcutilsLogger(name="furquan_debug")





# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force
        self.left_pub=self.create_publisher(
            Wrench,
            '/hb_bot_1/left_wheel_force',
            10
        )

        self.right_pub=self.create_publisher(
            Wrench,
            '/hb_bot_1/right_wheel_force',
            10
        )

        self.rear_pub=self.create_publisher(
            Wrench,
            '/hb_bot_1/rear_wheel_force',
            10
        )

        
        self.pose_sub=self.create_subscription(
            Pose2D,
            '/detected_aruco',
            self.update_pose, 10)
        
        self.posesub_x = 0
        self.posesub_y = 0
        self.posesub_theta = 0

        self.pose_msg = Pose2D()

        

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.kp = 0.5
        self.ka = 0.35
        self.flag = 0

        # client for the "next_goal" service
             
        
        self.next_goal_client = self.create_client(NextGoal, 'next_goal')

        self.rate = self.create_rate(100)
        # self.rate = self.next_goal_client.create_rate(1)

        while not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "next_goal" not available, waiting...')

        self.req = NextGoal.Request()

        self.x_g = [4, -4, -4, 4, 0]
        self.y_g = [4, 4, -4, -4, 0]
        # self.theta = [0, PI/2, PI, -PI/2, 0]
        self.theta = [0, 0, 0, 0, 0]

        self.index = 0
    
    # Method to create a request to the "next_goal" service
    # def send_request(self, request_goal):
    #     self.req.request_goal = request_goal
    #     self.future = self.cli.call_async(self.req)

   
    def update_pose(self, pose_msg):


        position = pose_msg
        (self.posesub_x, self.posesub_y) = (position.x, position.y )
        self.posesub_theta = position.theta
        logger.info(str(self.posesub_x) + str(self.posesub_y))


        

    def inverse_kinematics(self, x_g, y_g, theta):
        logger.info("goal received = "+str(x_g)+str(y_g)+str(theta))
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        while(rclpy.ok()):
            errox = x_g-self.posesub_x
            erroy = y_g-self.posesub_y
            errot = theta-self.posesub_theta

            print("Errors: "+ str(errox))
            velx = self.kp*errox
            vely = self.kp*erroy
            velz = self.kp*errot

            # TxMt= np.array([
            #             [int(-np.sin(PI/6 + theta)), 
            #             int(np.cos(PI/6+theta)),1],
            #             [int(-np.sin(PI/6 + theta)), 
            #             int(-np.cos(PI/6+theta)),1],
            #             [int(np.cos(theta)), 
            #             int(np.sin(theta)),1]])

            # adding 1 1 1 extra row to make this square matrix so it is invertable.

            # #force calc here
            # #[Fx, Fy] = [F1, F2, F3] * [TXMT]

            # [Fx, Fy, w] = [velx, vely, velz]

            # [F1, F2, F3] = np.matmul([Fx, Fy, w], np.linalg.inv(TxMt))

            #new IK::
            TxMV = np.array([[0.58, -0.33, 0.33], [-0.58, -0.33, 0.33], [0, 0.67, 0.33]])
            [F1, F2, F3] = np.matmul(TxMV, [velx, vely, velz])

            print("Forces : "+str([F1, F2, F3]))

            LeftMsg = Wrench()
            RightMsg = Wrench()
            RearMsg = Wrench()
            
            #seperate karna hai cause iterable nahi hai yeh.
            
            LeftMsg.force.x=F1
            RightMsg.force.x=F2
            RearMsg.force.x=F3

            self.left_pub.publish(LeftMsg)
            self.right_pub.publish(RightMsg)
            self.rear_pub.publish(RearMsg)


            if (abs(errox)<0.5 and abs(erroy))<0.5:
                time.sleep(2)
                logger.info("Goal has been reached.")
                velx=0
                vely=0

                # yeh if ka point hi nahi hai cause yeh stop nahi kar raha. Publish toh isse pehle ho raha hai toh velx vely =0 karke isne kuch nahi ukhada. same for task1b. 
                # 1b ke mein bhi change kar diya hu waise toh monday ko check karo kya scene hai 1b mein (if time) varna just fix this.

            rclpy.spin_once(self)
        logger.info("CL run done")
        
    def send_request(self, index):
        self.req.request_goal = index
        self.future = self.next_goal_client.call_async(self.req)
        rclpy.spin_once(self)
        self.get_logger().info('request sent here.')
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result
        self.get_logger().info('response is populated')
        return response


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():
        logger.info("ros ok")

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
                logger.info("try ok")
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal + 250
                y_goal      = response.y_goal + 250
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################

                hb_controller.inverse_kinematics(x_goal, y_goal, theta_goal)
                
                # Calculate Error from feedback

                # Change the frame by using Rotation Matrix (If you find it required)

                # Calculate the required velocity of bot for the next iteration(s)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)

                # Apply appropriate force vectors

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                        
                ############     DO NOT MODIFY THIS       #########
                hb_controller.index += 1
                if hb_controller.flag == 1 :
                    hb_controller.index = 0
                hb_controller.send_request(hb_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
