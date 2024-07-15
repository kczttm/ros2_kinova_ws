from .utilities import DeviceConnection
import numpy as np
np.set_printoptions(suppress=True)
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .tool_box import broadcast_world_EE_tf, broadcase_EE_endo_tf, get_tf
from .tool_box import get_endoscope_tf_from_yaml
from .tool_box import tf_to_hom_mtx
from .tool_box import TCPArguments, move_tool_pose_relative, get_world_EE_HomoMtx

from cv_bridge import CvBridge
from gen3_action_interfaces.action import TakePictures

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

class TakePicturesActionServer(Node):
    def __init__(self, base=None, base_cyclic=None):
        super().__init__('take_pictures_action_server')

        ## Kortex API declarations
        self.tcp_args = TCPArguments()
        self.base = base
        self.base_cyclic = base_cyclic
        
        # Action server for taking pictures
        self._action_server = ActionServer(
            self,
            TakePictures,
            'gen3_action/take_pictures',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback)
        self._current_goal = None
        
        # TF broadcaster
        self.tf_br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.EE_endo_tf = get_endoscope_tf_from_yaml()
        self.world_endo_tf = None
        
        self.image_sub = self.create_subscription(Image, 'endoscope/resize/image', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.image_data = None

    def image_callback(self, msg):
        self.image_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def capture_image(self):
        while self.image_data is None:
            rclpy.spin_once(self)
        img = self.image_data
        self.image_data = None
        return img
    
    def get_relative_displacement_in_world(self, p_endo, world_endo_tf):
        H_world_endo = tf_to_hom_mtx(world_endo_tf)
        p_world = np.dot(H_world_endo, np.append(p_endo, 1))
        return p_world[:3]
    

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Initial pose
        initial_image = self.capture_image()
        goal_handle.publish_feedback(TakePictures.Feedback(status='Captured first image'))

        # Move in meter to the right in endoscope frame
        x_offset = goal_handle.request.spacing
        p_endo = np.array([x_offset, 0, 0])

        # do the with router thing here
        with DeviceConnection.createTcpConnection(self.tcp_args) as router:
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(router)

            # In the future, try to implement a way to check if Session is in use
            # feedback = self.base_cyclic.RefreshFeedback()
            # is_robot_active = feedback.interconnect.status_flags
            # print('active?', is_robot_active)

            # Make sure the arm is in Single Level Servoing mode (high-level mode)
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)

            #####bypassing the tf routine for now since it's sooo not easy
            # # broadcast the both tf to perform the endo frame transformation of the p_endo
            # broadcast_world_EE_tf(self.base_cyclic, self, self.tf_br)
            # broadcase_EE_endo_tf(self, self.tf_br, self.EE_endo_tf)

            # # see if the EE frame is available
            # self.world_endo_tf = get_tf(self, self.tf_buffer, "world", "endoscope")
            # if self.world_endo_tf is None:
            #     return
            # p_world = self.get_relative_displacement_in_world(p_endo, self.world_endo_tf)
            # self.get_logger().info(f"Relative displacement in world frame: {p_world}")

            H_wd_ee = get_world_EE_HomoMtx(self.base)
            H_ee_endo = tf_to_hom_mtx(self.EE_endo_tf)
            H_wd_endo = H_wd_ee @ H_ee_endo
            p_world = np.dot(H_wd_endo, np.append(p_endo, 1))[:3]-H_wd_endo[:3,3]
            self.get_logger().info(f"Relative displacement in world frame: {p_world}")    

            # convert pose to kinova pose
            p_world_kinova = np.array([p_world[0], p_world[1], p_world[2], 0, 0, 0])

            # move the tool in the endoscope frame
            action_result = move_tool_pose_relative(self.base, self.base_cyclic, p_world_kinova)

            # wait for 0.5 second
            time.sleep(0.5)

            new_image = self.capture_image()
            goal_handle.publish_feedback(TakePictures.Feedback(status='Captured second image'))

            result = TakePictures.Result()

            result.images = [self.bridge.cv2_to_imgmsg(initial_image, encoding='bgr8'),
                            self.bridge.cv2_to_imgmsg(new_image, encoding='bgr8')]
            # move the tool back to the initial position
            action_result = move_tool_pose_relative(self.base, self.base_cyclic, -p_world_kinova)
        
        goal_handle.publish_feedback(TakePictures.Feedback(status='Finished with capturing images'))
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TakePicturesActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(node, executor=executor)
    rclpy.destroy_node(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
