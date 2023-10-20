#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from math import sqrt, atan2, pi
from nav_msgs.msg import Odometry

class GoToGoal(Node):
    pose_x=0.0
    pose_y=0.0
    theta=0.0
    def _init__(self):
        super().__init__("goto_goal")
        self.velocity_publisher_=self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer=self.create_timer(0.01, self.goto_goal)
        self.pose_subscriber_=self.create_subscription(Odometry, "/odom", self.pose_callback,10)
        self.set_goal()

    def set_goal(self):
        self.get_logger().info("set the desired x2 and y2 position")
        self.x2 = float(input("enter a desired x position : "))
        self.y2 = float(input("enter a desired y position : "))

    def pose_callback(self, pose_msg=Odometry()):
        self.pose_x=pose_msg.x
        self.pose_y=pose_msg.y
        self.theta=pose_msg.theta

    def goto_goal(self):
        dist_x=self.x2 - self.pose_x
        dist_y=self.y2 - self.pose_y
        euclidean_distance = sqrt(dist_x**2 + dist_y**2)
        distance_tolerance = 0.3

        speed=Twist()
        if (euclidean_distance > distance_tolerance):
            K_linear=2
            speed.linear.x= K_linear * euclidean_distance

            angle= atan2(dist_y, dist_x)
            diff = angle - self.theta
            K_angular=6

            if diff > pi:
                diff -=2*pi
            elif diff < -pi:    
                diff += 2*pi

            speed.angular.z= K_angular*diff

        else:
            speed.linear.x=0.0
            speed.angular.z=0.0
            self.get_logger().info("target reached")
            self.timer_.cancel()

        self.velocity_publisher_(speed)

def main(args=None):
    rclpy.init(args=args)        
    goto_goal = GoToGoal
    rclpy.spin(goto_goal)
    rclpy.shutdown() 

if __name__=='__main__':
    main()    








        