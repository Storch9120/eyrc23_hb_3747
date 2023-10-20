import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
PI = 3.14
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
from rclpy.impl import rcutils_logger
logger = rcutils_logger.RcutilsLogger(name="furquan_debug")


class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')

        #Pub and Sub
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.update_odom,
            10
        )

        # Declare a Twist message
        self.vel = Twist()
        # Initialise the required variables to 0
        self.odom_x=0
        self.odom_y=0
        self.odom_t=0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller
        self.kp=0.35
        self.ka=0.35
        self.flag=0

        self.next_goal_client = self.create_client(NextGoal, 'next_goal')

        self.rate = self.create_rate(100)
        # self.rate = self.next_goal_client.create_rate(1)

        while not self.next_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "next_goal" not available, waiting...')

        self.req = NextGoal.Request()

        # client for the "next_goal" service
        # self.cli = self.create_client(NextGoal, 'next_goal')      
        # self.req = NextGoal.Request() 
        self.x_g = [4, -4, -4, 4, 0]
        self.y_g = [4, 4, -4, -4, 0]
        # self.theta = [0, PI/2, PI, -PI/2, 0]
        self.theta = [0, 0, 0, 0, 0]

        self.index = 0

    def update_odom(self, msg):
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        position = msg.pose.pose.position
        # orientation = msg2.pose.pose.orientation
        (self.odom_x, self.odom_y) = (position.x, position.y)
        self.odom_t = yaw 
        # (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        # similarly for twist message if you need

    def control_loop(self, x_g, y_g, theta):
        logger.info("goal received = "+str(x_g)+str(y_g)+str(theta))
        while(rclpy.ok()):
            # logger.info(str(self.odom_x)+" x --- y "+str(self.odom_y)+"-- t"+ str(self.odom_t))

            errox = x_g-self.odom_x
            erroy = y_g-self.odom_y
            errot = theta-self.odom_t

            ka = 0.1

            self.vel.linear.x = self.kp*errox
            self.vel.linear.y = self.kp*erroy
            self.vel.angular.z = self.kp*errot
            # logger.info(str(errox)+" x --- y "+str(erroy))
            # Publish control commands
            self.cmd_vel_publisher.publish(self.vel)
            # and abs(errot)<1.5 +"-- t"+ str(errot)
            if (abs(errox) <0.5 and abs(erroy)<0.5 ) :
                time.sleep(2)
                logger.info("Goal has been reached.")
                self.vel.linear.x=0.0
                self.vel.linear.y=0.0
                break
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
    
    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()
   
    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)
    
    # Main loop
    while rclpy.ok():
        logger.info("ros ok")
        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
                logger.info("try ok")
            except Exception as e:
                ebot_controller.get_logger().infselfo('Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################

                ebot_controller.control_loop(x_goal, y_goal, theta_goal)

                # Find error (in x, y and theta) in global frame
                # the /odom topic is giving pose of the robot in global frame
                # the desired pose is declared above and defined by you in global frame
                # therefore calculate error in global frame

                # (Calculate error in body frame)
                # But for Controller outputs robot velocity in robot_body frame, 
                # i.e. velocity are define is in x, y of the robot frame, 
                # Notice: the direction of z axis says the same in global and body frame
                # therefore the errors will have have to be calculated in body frame.
                # 
                # This is probably the crux of Task 1, figure this out and rest should be fine.

                # Finally implement a P controller 
                # to react to the error with velocities in x, y and theta.

                # Safety Check
                # make sure the velocities are within a range.
                # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
                # we may get away with skipping this step. But it will be very necessary in the long run.


                #If Condition is up to you
                
                ############     DO NOT MODIFY THIS       #########
                ebot_controller.index += 1
                if ebot_controller.flag == 1 :
                    ebot_controller.index = 0
                ebot_controller.send_request(ebot_controller.index)
                ####################################################

    # Spin once to process callbacks
    rclpy.spin_once(ebot_controller)
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
