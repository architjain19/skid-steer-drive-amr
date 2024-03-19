#! /usr/bin/env python3

# *****************************************************************************************
# *
# *  Filename:			amr_control.py
# *  Created:			20/03/2024
# *  Last Modified:	    20/03/2024
# *  Author:			Archit Jain
# *  
# *****************************************************************************************

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

class AmrNavigate(Node):
    def __init__(self):
        super().__init__('amr_nav_node')
        self.callback_group = ReentrantCallbackGroup()


        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def odometry_callback(self, msg):
        self.odom_msg = msg

        self.amr_pose[0] = msg.pose.pose.position.x
        self.amr_pose[1] = msg.pose.pose.position.y

        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _,_, yaw = euler_from_quaternion(orientation_list)

        self.amr_pose[2] = yaw

def main(args=None):
    rclpy.init(args=args)
    amr_nav_node = AmrNavigate()
    rclpy.spin(amr_nav_node)
    amr_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()