########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID: 3747
# Team Leader Name: Shiekh Furquan M Fayiaz
# Author List: Shiekh Furquan M Fayiaz
# Team Members Name: Shiekh Furquan M Fayiaz, Himanshu Singh, Satvik Sharma, Brajesh Patil
# College: Veermata Jeejabai Technological Institute (VJTI)
# Filename: task_1a_3747.py
# Functions:  __init__(), pose_callback1(), pose_callback2(), stop_turtle, spawn_turtle, main()
########################################################################################################################

#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class Task_1a_3747(Node):
    def __init__(self):
         # Initialize the node with a unique name

        super().__init__("task_1a_3747")

        # defining the name of the turtles
        self.turtle1= '/turtle1'
        self.turtle2= '/turtle2'

         # Create publishers for velocity commands for turtles
        self.vel1_pub=self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.vel2_pub=self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

         # Create subscribers for turtle pose information
        self.pose_subscriber_turtle_1=self.create_subscription(Pose, "/turtle1/pose", self.pose_callback1, 10)
        self.pose_subscriber_turtle_2=self.create_subscription(Pose, "/turtle2/pose", self.pose_callback2, 10)

        # Log node initialization
        self.get_logger().info("node has been started")

        # Initialize turtle orientations
        self.theta1=0.0
        self.theta2=0.0

    # Callback function for turtle1's pose
    def pose_callback1(self, msg): 
       self.theta1=msg.theta
      
       if -0.2<=self.theta1<0.0:
            
    # Stop turtle1, spawn turtle2, and start tracking turtle2's pose
            self.stop_turtle(self.turtle1)
            self.spawn_turtle()
            self.create_subscription(Pose, "/turtle2/pose", self.pose_callback2, 10)
            self.turtle2_active = True
       else:
            
            # Move turtle1 forward and make the upper circle
            twist = Twist()
            twist.linear.x = 1.0
            twist.angular.z = 1.0
            self.vel1_pub.publish(twist)


    # Callback function for turtle2's pose
    def pose_callback2(self, data): 
        self.theta2 = data.theta


        # Stop turtle2 if it completes a circle
        if 0.031<self.theta2 <0.2:
            self.stop_turtle(self.turtle2)
        else:
        
        # Move turtle2 to form a circle
            twist = Twist()
            twist.linear.x = 2.0
            twist.angular.z = -1.0
            self.vel2_pub.publish(twist)   

    # Function to stop a turtle's movement
    def stop_turtle(self, turtle_name)       :
        twist=Twist()
        twist.linear.x=0.0
        twist.angular.z=0.0

        if turtle_name == self.turtle1:
            self.vel1_pub.publish(twist)
        elif turtle_name == self.turtle2:
            self.vel2_pub.publish(twist)                   

    # Function to spawn turtle2
    def spawn_turtle(self):
        spawn_client=self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the "spawn" service...')
            
        request = Spawn.Request()
        request.name = self.turtle2
        request.x = 5.197916507720947
        request.y = 5.197916507720947
        request.theta = 0.0
        spawn_client.call_async(request)        


# Main function
def main(args=None):
    rclpy.init(args=args)
    node=Task_1a_3747()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


