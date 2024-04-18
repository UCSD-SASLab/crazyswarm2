#!/usr/bin/env python
from crazyflie_interfaces.srv import GoTo, Land, Takeoff, NotifySetpointsStop
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import numpy as np

import rclpy 
import rclpy.node
import rowan
from crazyflie_py import Crazyswarm


K_matrix = np.array([[0.0, 0.4, 0.0, 0.0, 0.4, 0.0, 0.0],
                     [-0.4, 0.0, 0.0, -0.4, 0.0, 0.0, 0.0], 
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.4721],  
                     [0.0, 0.0, -5.4772, 0.0, 0.0, -5.5637, 0.0]])

u_target = np.array([0.0, 0.0, 0.0, 10.5])
x_target = np.array([-4.5, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0])


class FeedbackController(Crazyswarm):  # Might need to have it be a child class of Crazyswarm instead?
    def __init__(self):
        super().__init__()  # Inits Crazyswarm
        self.rclpy_node = self.allcfs
        self.rclpy_node.estimated_state = np.zeros(7) # TODO: Customize for different state fidelities
        self.rclpy_node.lqr_active = False
        assert len(self.rclpy_node.crazyflies) == 1, "Feedback controller only supports one drone"
        for cf in self.rclpy_node.crazyflies:
            cf_name = cf.prefix
            self.odomSubscriber = self.rclpy_node.create_subscription(Odometry, f'{cf_name}/odom', self.odom_callback, 10)
        self.num_zeros_sent = 0.0
        self.start_lqr_subscriber = self.rclpy_node.create_subscription(String, 'start_lqr', self.start_lqr_callback, 10)

    def start_lqr_callback(self, msg):
        self.rclpy_node.lqr_active = not self.rclpy_node.lqr_active
        if self.rclpy_node.lqr_active:
            print("LQR activated")
        else:
            print("End of LQR, landing")
            for cf in self.rclpy_node.crazyflies:
                cf.notifySetpointsStop(10)
            self.rclpy_node.land(targetHeight=0.06, duration=5.0)
            self.timeHelper.sleep(0.1)
            self.rclpy_node.land(targetHeight=0.06, duration=5.0)
   
    def odom_callback(self, msg):
        # Read odom msg and update estimated state (x, y, z, xdot, ydot, zdot, yaw)
        new_state = [msg.pose.pose.position.x, 
                     msg.pose.pose.position.y, 
                     msg.pose.pose.position.z, 
                     msg.twist.twist.linear.x, 
                     msg.twist.twist.linear.y, 
                     msg.twist.twist.linear.z,
                     msg.pose.pose.orientation.z]
        self.rclpy_node.estimated_state = new_state
        self.update_control()  # TODO: make such that this is controlled with separate timer

    def update_control(self):
        # Calculate control input based on estimated state
        # Publish control input
        if self.rclpy_node.lqr_active:
            if self.num_zeros_sent < 5:
                converted_control = np.zeros(4)
                self.num_zeros_sent += 1
            else:
                control = K_matrix @ (self.rclpy_node.estimated_state - x_target) + u_target
                np.set_printoptions(precision=2)
                print("state", self.rclpy_node.estimated_state)
                print("control", control)
                converted_control = self.convert_control(control)
            for cf in self.rclpy_node.crazyflies:
                cf.cmdVel(converted_control[0], converted_control[1], converted_control[2], converted_control[3])
        
    @staticmethod
    def convert_control(control):
        # rpy to degrees, thrust scaled to 0-65535 from 0-16
        # rpy = control[:3] * 180 / np.pi
        rpy = control[:3]
        thrust = control[3] * 4096
        return np.array([rpy[0], rpy[1], rpy[2], thrust])   
