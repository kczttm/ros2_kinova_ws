import rclpy
from rclpy.node import Node

import time

from geometry_msgs.msg import Pose, TransformStamped

from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .tool_box import get_endoscope_tf_from_yaml, get_desired_endoscope_pose_from_tag, get_inverse_tf

class TagFollowerNode(Node):
    def __init__(self):
        super().__init__('apriltag_follower_node')
        # follow the apriltag detected from endoscope using gen3_7dof
        # node will start 3 seconds after the launch file is executed

        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10)
        self.pos_pub = self.create_publisher(
            Pose, 
            'gen3_7dof/desired_EE_pose_topic',
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
       
        self.tf_br = TransformBroadcaster(self)
        # self.static_tf_br = StaticTransformBroadcaster(self)

        self.feedback_period = 0.025  # 40 Hz high-level control loop freq
        self.timer = self.create_timer(self.feedback_period, self.post_desired_ee_from_tag)

        self.EE_to_endo = None

        self.follow_the_tag = True
        self.desired_tag_id = 'tag36h11:581'  # inthe future can get this from yaml file
        self.desired_endo = None  # see if static tf has been posted
            

    def post_desired_ee_from_tag(self):
        # static tf from tag frame to desired_endo frame
        self.desired_endo = get_desired_endoscope_pose_from_tag()
        self.desired_endo.header.stamp = self.get_clock().now().to_msg()
        self.desired_endo.header.frame_id = self.desired_tag_id  # currently follow the last tag
        self.tf_br.sendTransform(self.desired_endo)

        # static tf from desired_end_effector frame to desired_endo frame
        endo_to_ee_tf = get_endoscope_tf_from_yaml()
        desired_ee = get_inverse_tf(endo_to_ee_tf)
        desired_ee.header.stamp = self.get_clock().now().to_msg()
        desired_ee.header.frame_id = self.desired_endo.child_frame_id
        desired_ee.child_frame_id = "desired_end_effector"
        self.tf_br.sendTransform(desired_ee)

    
    def get_tf(self, source_frame, target_frame):
        # get the tf described in source frame coordinates
        try:
            now = rclpy.time.Time()
            # till = rclpy.duration.Duration(seconds=1.0)
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, now)
            return get_inverse_tf(tf)
        except TransformException as e:
            self.get_logger().error(f"Failed to get tf from {source_frame} to {target_frame}")
            return None
        

    def publish_desired_ee(self, desired_ee_in_world):
        desired_ee = Pose()
        desired_ee.position.x = desired_ee_in_world.transform.translation.x
        desired_ee.position.y = desired_ee_in_world.transform.translation.y
        desired_ee.position.z = desired_ee_in_world.transform.translation.z
        desired_ee.orientation.x = desired_ee_in_world.transform.rotation.x
        desired_ee.orientation.y = desired_ee_in_world.transform.rotation.y
        desired_ee.orientation.z = desired_ee_in_world.transform.rotation.z
        desired_ee.orientation.w = desired_ee_in_world.transform.rotation.w
        self.pos_pub.publish(desired_ee)

        
    def tag_callback(self, msg):
        ## need to write a another file that post desired EE pose to drive robot
        tag_found = False
        
        for detection in msg.detections:
            tag_family = detection.family
            tag_id = detection.id
            tag_frame_id = f'{tag_family}:{tag_id}'
            tag_found = tag_frame_id == self.desired_tag_id
            if tag_found:
                tf_exist = self.tf_buffer.can_transform(
                    "world", "desired_end_effector", rclpy.time.Time())
                # if tf_exist:
                #     print(self.get_tf("world","desired_end_effector"))
                if self.follow_the_tag and tf_exist:
                    desired_ee_in_world = self.get_tf("world", "desired_end_effector")
                    if desired_ee_in_world is not None:
                        self.publish_desired_ee(desired_ee_in_world)
                    else:
                        self.get_logger().error("Failed to move the robot")
                break
                    

def main(args=None):
    rclpy.init(args=args)
    tag_follower_node = TagFollowerNode()
    try:
        rclpy.spin(tag_follower_node)
    except KeyboardInterrupt:
        pass  # might implement a robot stopping mechanism here
    tag_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()