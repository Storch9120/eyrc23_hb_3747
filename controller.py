#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import time
import math

##ssharma 10-10-23
## Obj: To correct the service response parsing. 
# Program Desc: The controller pushes twist to /cmd_vel  and P controller uses feedback 
# from /odom. The service NextGoal is requested to set the goal which the bot 
# is to achieve. 


class HBTask1BController(Node):
    def __init__(self):
        super().__init__('hb_task1b_controller')

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.vel = Twist()
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0

        # self.x_goals = [4, -4, -4, 4, 0]
        # self.y_goals = [4, 4, -4, -4, 0]
        # self.theta_goals = [0, math.pi/2, math.pi, -math.pi/2, 0]

        
        self.rate = self.create_rate(100)

        # Create a client for the "next_goal" service
        self.next_goal_client = self.create_client(NextGoal, 'next_goal')

        while not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "next_goal" not available, waiting...')

        self.req = NextGoal.Request()
        self.index = 0

    def odometry_callback(self, msg):
        print("odom was called")
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        self.hb_theta = yaw

    def send_request(self, index):
        self.req.request_goal = index
        future = self.next_goal_client.call_async(self.req)
        rclpy.spin_once(self)
        if future.done():
            try:
                response = future.result()
                return response
            except Exception as e:
                self.get_logger().info(f'Service call failed: {e}')
                return None

    def control_loop(self, x, y, theta):
            x_goal = x
            y_goal = y
            theta_goal = theta

            error_x = x_goal - self.hb_x
            error_y = y_goal - self.hb_y
            error_theta = theta_goal - self.hb_theta

            # Calculate control commands (P controller)
            k_p_linear = 0.5  # Adjust this gain as needed
            k_p_angular = 1.0  # Adjust this gain as needed

            self.vel.linear.x = k_p_linear * error_x
            self.vel.linear.y = k_p_linear * error_y
            self.vel.angular.z = k_p_angular * error_theta

            # Publish control commands
            self.cmd_vel_publisher.publish(self.vel)

            # Check if the goal has been reached
            if (abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < 0.1):
                # Stay at the goal pose for at least 1 second
                time.sleep(1)
                # self.index += 1
                # if self.index >= len(self.x_goals):
                #     self.index = 0
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()
# Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)

    while rclpy.ok():
        #check if service call is done
        if ebot_controller.future.done():
            try:
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo('Service call failed %r' % (e,))
            else:
            #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
            ####################################################

            ############     DO NOT MODIFY THIS       #########
                ebot_controller.index += 1
                if ebot_controller.flag == 1 :
                    ebot_controller.index = 0
                ebot_controller.send_request(ebot_controller.index)
            ####################################################
    # controller.control_loop()
    # rclpy.spin(controller)
    rclpy.spin_once(ebot_controller)
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
