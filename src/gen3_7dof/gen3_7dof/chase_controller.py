import sys
import os
import time
import threading
from .utilities import DeviceConnection
import numpy as np
np.set_printoptions(suppress=True)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster
#from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .tool_box import broadcast_world_EE_tf, broadcase_EE_endo_tf
from .tool_box import get_endoscope_tf_from_yaml
from .tool_box import getRotMtx, R2rot, quaternion_to_euler, euler_to_quaternion 
from .tool_box import TCPArguments

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


class ChaseController(Node):
    def __init__(self, base, base_cyclic=None):
        super().__init__('chase_controller_node')
        ## Kortex API declarations
        self.base = base
        self.base_cyclic = base_cyclic

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
        
        ## ROS2 Nodes declarations
        self.pos_sub = self.create_subscription(
            Pose,
            'gen3_7dof/desired_EE_pose_topic',
            self.desired_pose_callback,
            10)
        self.pos_sub  # prevent unused variable warning
        
        self.feedback_period = 0.025  # 40 Hz high-level control loop freq
        self.timer = self.create_timer(self.feedback_period, self.move_to_pose)
        self.pos_pub = self.create_publisher(Pose,'gen3_7dof/EE_pose_topic',10)
        
        ## ROS2 TF declarations
        self.tf_br = TransformBroadcaster(self)

        # Post Static TF from EE to Endoscope
        self.static_tf = get_endoscope_tf_from_yaml()
        #static_tf_br = StaticTransformBroadcaster(self)
        #static_tf_br.sendTransform(static_tf)
        
        ## Control Parameters
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
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        ori_rpy = quaternion_to_euler(quat)
        self.desired_pose.theta_x = ori_rpy[0] * 180 / np.pi
        self.desired_pose.theta_y = ori_rpy[1] * 180 / np.pi
        self.desired_pose.theta_z = ori_rpy[2] * 180 / np.pi
        self.R_d = getRotMtx(self.desired_pose)

    
    def broadcast_pose_and_tf(self, current_pose):
        # Broadcast the transform from the base frame to the end-effector frame
        th_x_rad, th_y_rad, th_z_rad = current_pose.theta_x*np.pi/180, current_pose.theta_y*np.pi/180, current_pose.theta_z*np.pi/180
        current_quat = euler_to_quaternion(th_x_rad, th_y_rad, th_z_rad)
        # publish feedback pose
        feedback_pose = Pose()
        feedback_pose.position.x = current_pose.x
        feedback_pose.position.y = current_pose.y
        feedback_pose.position.z = current_pose.z
        feedback_pose.orientation.x = current_quat[0]
        feedback_pose.orientation.y = current_quat[1]
        feedback_pose.orientation.z = current_quat[2]
        feedback_pose.orientation.w = current_quat[3]
        self.pos_pub.publish(feedback_pose)

        # broadcast tf
        broadcast_world_EE_tf(self.base_cyclic, self, self.tf_br)

        # send static tf from end_effector to endoscope
        broadcase_EE_endo_tf(self, self.tf_br, self.static_tf)


    def move_to_pose(self):
        current_pose = self.base.GetMeasuredCartesianPose()
        R = getRotMtx(current_pose)
        
        self.broadcast_pose_and_tf(current_pose)

        # reducing the pos error (in global frame!!)
        pos_diff = np.array([self.desired_pose.x-current_pose.x,
                             self.desired_pose.y-current_pose.y,
                             self.desired_pose.z-current_pose.z])
        pos_diff_norm = np.linalg.norm(pos_diff)+1e-5
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
            # print("Goal Pose reached")
            self.base.Stop()
        else: 
            # print the current error
            self.get_logger().info('pos_diff_norm: %f, eR2_norm: %f' % (pos_diff_norm, eR2_norm))
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
        base_cyclic = BaseCyclicClient(router)
        robot_controller = ChaseController(base, base_cyclic)
        rclpy.spin(robot_controller)
    
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()