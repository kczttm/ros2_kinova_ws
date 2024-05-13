import sys
import os
import time
import threading
from .utilities import DeviceConnection
import numpy as np
np.set_printoptions(suppress=True)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .tool_box import TCPArguments, getRotMtx, R2rot, quaternion_to_euler  

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


class ChaseController(Node):
    def __init__(self, base):
        super().__init__('chase_controller')
        self.base = base
        # Make sure the arm is in Single Level Servoing mode (high-level mode)
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        # kortex twist control mode
        self.command = Base_pb2.TwistCommand()
        # note that twist is naturally in tool frame, but this conversion made things a bit easy
        self.command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        self.command.duration = 0

        # initialize command storage
        self.twist = self.command.twist
        self.twist.linear_x = 0.0  # adjust linear velocity
        self.twist.linear_y = 0.0
        self.twist.linear_z = 0.0
        self.twist.angular_x = 0.0
        self.twist.angular_y = 0.0
        self.twist.angular_z = 0.0  # adjust angular velocity
        
        self.pos_sub = self.create_subscription(
            Pose,
            'desired_pose_topic',
            self.desired_pose_callback,
            10)
        self.pos_sub  # prevent unused variable warning
        
        self.feedback_period = 0.025  # 40 Hz high-level control loop freq
        self.timer = self.create_timer(self.feedback_period, self.move_to_pose)
        # self.pos_pub = self.create_publisher(Pose
        
        # assign initial desired pose as current pose
        self.desired_pose = base.GetMeasuredCartesianPose()
        self.R_d = getRotMtx(self.desired_pose)

        # control constants
        self.max_vel = 0.5-0.1  # m/s  0.5 is max
        #max_vel = 0.1
        self.max_w = 70.0  # ~ 50 deg/s
        self.kp_pos = 2.5
        self.kp_ang = 4.0
        
        self.dcc_range = self.max_vel / (self.kp_pos * 2)  # dcc_range should be smaller than max_vel/kp_pos
        self.ang_dcc_range = self.max_w / (self.kp_ang * 6)

        self.eps_pos = 0.001  # convergence criterion
        self.eps_ang = 0.01
        

    def desired_pose_callback(self, msg):
        # Extract desired pose from the received message
        self.desired_pose.x = msg.position.x
        self.desired_pose.y = msg.position.y
        self.desired_pose.z = msg.position.z

        # convert orientation
        ori_rpy = quaternion_to_euler(msg.orientation)
        self.desired_pose.theta_x = ori_rpy[0] * 180 / np.pi
        self.desired_pose.theta_y = ori_rpy[1] * 180 / np.pi
        self.desired_pose.theta_z = ori_rpy[2] * 180 / np.pi
        self.R_d = getRotMtx(self.desired_pose)


    def move_to_pose(self):
        current_pose = self.base.GetMeasuredCartesianPose()
        R = getRotMtx(current_pose)

        # reducing the pos error (in global frame!!)
        pos_diff = np.array([self.desired_pose.x-current_pose.x,
                             self.desired_pose.y-current_pose.y,
                             self.desired_pose.z-current_pose.z])
        pos_diff_norm = np.linalg.norm(pos_diff)
        v_temp = self.max_vel * pos_diff/pos_diff_norm

        # reducing ang error using ER = RRd^T
        ER = R @ self.R_d.T
        # frobenius norm of matrix squre root of ER or eR2
        k,theta = R2rot(ER)
        k=np.array(k)
        eR2=-np.sin(theta/2)*k * 180 / np.pi  # not the best name but works fine
        
        eR2_norm = np.linalg.norm(eR2)+1e-5
        w_temp = self.max_w * eR2/eR2_norm

        # print(pos_diff_norm, eR2_norm)

        reached = pos_diff_norm < self.eps_pos and eR2_norm < self.eps_ang
        
        if reached:
            print("Goal Pose reached")
            self.base.Stop()
        else: 
            # go in max vel when outside dcc_range
            if pos_diff_norm < self.dcc_range:
                v = self.kp_pos * pos_diff
            else:
                v = v_temp
    
            if eR2_norm < self.ang_dcc_range:
                kR = self.kp_ang*np.eye(3)
                w = kR @ eR2 
            else:
                w = w_temp
                
            self.twist.linear_x = v[0]
            self.twist.linear_y = v[1] 
            self.twist.linear_z = v[2]
            self.twist.angular_x = w[0]
            self.twist.angular_y = w[1]
            self.twist.angular_z = w[2]
            self.base.SendTwistCommand(self.command)
        

def main(args=None):
    rclpy.init(args=args)
    tcp_args = TCPArguments()
    with DeviceConnection.createTcpConnection(tcp_args) as router:
        # Create connection services
        base = BaseClient(router)
        robot_controller = ChaseController(base)
        rclpy.spin(robot_controller)
    
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()