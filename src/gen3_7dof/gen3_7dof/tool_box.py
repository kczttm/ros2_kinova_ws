# Toolbox for Kinova Gen3 7 dof robot
# Put-togethered by: Chuizheng Kong
# Last Edited: 2024-06-13

import os, yaml, time, threading
import numpy as np
import rclpy

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

##**************Catalog of Functions**************
## 1. TF2 Related Codes
## 2. Interfacing Configs
## 3. Rotation Representation Conversions
## 4. Robot Command Functions
##************************************************

######################### TF2 Related Codes #########################
def broadcase_EE_endo_tf(active_node, tf_br, EE_endo_tf):
    """
    Broadcast the end effector to endoscope transformation to the TF tree.
    
    Args:
        kinova_base_cyclic (BaseCyclicClient): Kinova Gen3 7 dof base cyclic client.
        active_node (rclpy.node.Node): Active ROS2 node.
        tf_br (tf2_ros.TransformBroadcaster): TransformBroadcaster object.
        EE_endo_tf (geometry_msgs.msg.TransformStamped): output of the get_endoscope_tf_from_yaml().
    """

    EE_endo_tf.header.stamp = active_node.get_clock().now().to_msg()
    tf_br.sendTransform(EE_endo_tf)


def broadcast_world_EE_tf(kinova_base_cyclic, active_node, tf_br):
    """
    Broadcast the end effector transformation to the TF tree.
    
    Args:
        kinova_base_cyclic (BaseCyclicClient): Kinova Gen3 7 dof base cyclic client.
        active_node (rclpy.node.Node): Active ROS2 node.
        tf_br (tf2_ros.TransformBroadcaster): TransformBroadcaster object.
        tf (geometry_msgs.msg.TransformStamped): TransformStamped message.
    """
    # Get the current end effector pose
    feedback = kinova_base_cyclic.RefreshFeedback()
    th_x_rad = feedback.base.tool_pose_theta_x
    th_y_rad = feedback.base.tool_pose_theta_y
    th_z_rad = feedback.base.tool_pose_theta_z
    feedback_quat = euler_to_quaternion(th_x_rad, th_y_rad, th_z_rad)

    tf = TransformStamped()
    tf.header.stamp = active_node.get_clock().now().to_msg()
    tf.header.frame_id = "world"
    tf.child_frame_id = "end_effector"
    tf.transform.translation.x = feedback.base.tool_pose_x
    tf.transform.translation.y = feedback.base.tool_pose_y
    tf.transform.translation.z = feedback.base.tool_pose_z
    tf.transform.rotation.x = feedback_quat[0]
    tf.transform.rotation.y = feedback_quat[1]
    tf.transform.rotation.z = feedback_quat[2]
    tf.transform.rotation.w = feedback_quat[3]
    tf_br.sendTransform(tf)


def get_tf(active_node, tf_buffer, source_frame, target_frame):
    # get the tf described in source frame coordinates
    active_node.get_logger().info(f"Searching for tf from {source_frame} to {target_frame}")
    rclpy.spin_once(active_node)
    try:
        now = rclpy.time.Time()
        till = rclpy.duration.Duration(seconds=15.0)
        tf = tf_buffer.lookup_transform(
            target_frame, source_frame, now, till)
        return get_inverse_tf(tf)
    except TransformException as e:
        active_node.get_logger().error(f"Failed to get tf from {source_frame} to {target_frame}")
        return None


def get_inverse_tf(tf):
    """
    Get the inverse of a TransformStamped message.
    
    Args:
        tf (geometry_msgs.msg.TransformStamped): TransformStamped message.
    
    Returns:
        geometry_msgs.msg.TransformStamped: Inverse of the input TransformStamped message.
    """
    p, q = tf_to_vectors(tf)
    inv_q = [-q[0], -q[1], -q[2], q[3]]
    inv_p = -quaternion_rotate_vector(inv_q, p)
    inv_tf = TransformStamped()
    inv_tf.header.frame_id = tf.child_frame_id
    inv_tf.child_frame_id = tf.header.frame_id
    inv_tf.transform.translation.x = inv_p[0]
    inv_tf.transform.translation.y = inv_p[1]
    inv_tf.transform.translation.z = inv_p[2]
    inv_tf.transform.rotation.x = inv_q[0]
    inv_tf.transform.rotation.y = inv_q[1]
    inv_tf.transform.rotation.z = inv_q[2]
    inv_tf.transform.rotation.w = inv_q[3]
    return inv_tf


def tf_to_vectors(tf):
    """
    Convert a TransformStamped message to position and quaternion vectors.
    
    Args:
        tf (geometry_msgs.msg.TransformStamped): TransformStamped message.
    
    Returns:
        numpy.array: Position vector [x, y, z].
        numpy.array: Orientation vector [qx,qy,qz,qw].
    """
    # Extract the position from the TransformStamped message
    pos = np.array([
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z
    ])
    
    # Extract the orientation from the TransformStamped message
    q = np.array([
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w
    ])
    return pos, q


def tf_to_hom_mtx(tf):
    """
    Convert a TransformStamped message to a 4x4 homogeneous matrix.
    
    Args:
        tf (geometry_msgs.msg.TransformStamped): TransformStamped message.
    
    Returns:
        numpy.array: 4x4 homogeneous matrix.
    """
    # Extract the position and quaternion from the TransformStamped message
    p, q = tf_to_vectors(tf)
    
    # Compute the homogeneous matrix from the position and quaternion
    R = tf_to_rot_mtx(tf)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T



def tf_to_rot_mtx(tf):
    """
    Convert a TransformStamped message to a 3x3 rotation matrix.
    
    Args:
        tf (geometry_msgs.msg.TransformStamped): TransformStamped message.
    
    Returns:
        numpy.array: 3x3 rotation matrix.
    """
    # Extract the quaternion from the TransformStamped message
    q = [tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w]
    
    # Compute the rotation matrix from the quaternion
    ori_rpy = quaternion_to_euler(q)
    R = euler_to_rotation_matrix(ori_rpy[0], ori_rpy[1], ori_rpy[2])
    
    return R


######################### Interfacing Configs #########################

def get_realsense_on_link1_HomoMtx(base):
    # use the drawing dimension of the base to first link and 
    # the drawings of the camera and
    # the current measured angle of the first link 
    # to get the homogeneous matrix

    p_base_to_link1 = np.array([0.0, 0.0, 0.1564]) # meters
    p_link1_to_camera = np.array([0.03005, 0.0762, 0.1125])
    # note that instead of the using the link1's own frame we extend the base frame on this
    joints = base.GetMeasuredJointAngles()
    first_joint_angle = joints.joint_angles[0].value  # note in degrees
    R_base_link1 = euler_to_rotation_matrix(0, 0, -np.radians(first_joint_angle))
    p_base_to_camera = p_base_to_link1 + R_base_link1 @ p_link1_to_camera
    R_link1_camera = euler_to_rotation_matrix(-np.radians(90), 0, -np.radians(90))
    # account for the ~10 degree error on the mount about the y axis
    R_link1_camera = R_link1_camera @ euler_to_rotation_matrix(0, np.radians(0.5), 0)
    
    R_base_camera = R_base_link1 @ R_link1_camera
    H_base_camera = np.eye(4)
    H_base_camera[:3, :3] = R_base_camera
    H_base_camera[:3, 3] = p_base_to_camera
    return H_base_camera


def get_endoscope_tf_from_yaml():
    """
    Get the endoscope to EE transformation from a yaml file.
    If the file doesn't exist, use a default transformation. 
    The output is a TransformStamped message.   
    """
    pkg_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__),'..'))
    config_dir = os.path.join(pkg_dir, 'config/')
    fname = "endoscope_from_ee.yaml"
    
    try:
        with open(config_dir+fname, "r") as file:
            data = yaml.safe_load(file)

    except FileNotFoundError:
        print("File not found. Using default transformation.")
        # camera frame is 180 degree rotated around z axis of end effector frame
        # camera origin is 0.11m away from the end effector +z axis
        # camera origin is 0.05m away from the end effector -y axis
        data = {
            "header": {
                "frame_id": "end_effector"
            },
            "child_frame_id": "endoscope",
            "transform": {
                "translation": {
                    "x": 0.0,
                    "y": -0.05,
                    "z": 0.11
                },
                "rotation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.0, 
                    "w": 0.0
                }
            }
        }
        with open(config_dir+fname, "w") as file:
            yaml.safe_dump(data, file)
    except yaml.YAMLError as exc:
        print(exc)

    endoscope_tf = TransformStamped()
    endoscope_tf.header.frame_id = data["header"]["frame_id"]
    endoscope_tf.child_frame_id = data["child_frame_id"]
    endoscope_tf.transform.translation.x = data["transform"]["translation"]["x"]
    endoscope_tf.transform.translation.y = data["transform"]["translation"]["y"]
    endoscope_tf.transform.translation.z = data["transform"]["translation"]["z"]
    endoscope_tf.transform.rotation.x = data["transform"]["rotation"]["x"]
    endoscope_tf.transform.rotation.y = data["transform"]["rotation"]["y"]
    endoscope_tf.transform.rotation.z = data["transform"]["rotation"]["z"]
    endoscope_tf.transform.rotation.w = data["transform"]["rotation"]["w"]
    return endoscope_tf


def get_polli_fork_tf_from_yaml():
    """
    Get the polli fork to EE transformation from a yaml file.
    If the file doesn't exist, use a default transformation. 
    The output is a TransformStamped message.   
    """
    pkg_dir = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__),'..'))
    config_dir = os.path.join(pkg_dir, 'config/')
    fname = "polli_fork_from_ee.yaml"
    
    try:
        with open(config_dir+fname, "r") as file:
            data = yaml.safe_load(file)

    except FileNotFoundError:
        print("File not found. Using default transformation.")
        # camera frame is 180 degree rotated around z axis of end effector frame
        # camera origin is 0.155m away from the end effector +z axis
        # camera origin is 0.065m away from the end effector -y axis
        data = {
            "header": {
                "frame_id": "end_effector"
            },
            "child_frame_id": "polli_fork",
            "transform": {
                "translation": {
                    "x": 0.0,
                    "y": -0.065,
                    "z": 0.155
                },
                "rotation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.0, 
                    "w": 0.0
                }
            }
        }
        with open(config_dir+fname, "w") as file:
            yaml.safe_dump(data, file)
    except yaml.YAMLError as exc:
        print(exc)

    tf = TransformStamped()
    tf.header.frame_id = data["header"]["frame_id"]
    tf.child_frame_id = data["child_frame_id"]
    tf.transform.translation.x = data["transform"]["translation"]["x"]
    tf.transform.translation.y = data["transform"]["translation"]["y"]
    tf.transform.translation.z = data["transform"]["translation"]["z"]
    tf.transform.rotation.x = data["transform"]["rotation"]["x"]
    tf.transform.rotation.y = data["transform"]["rotation"]["y"]
    tf.transform.rotation.z = data["transform"]["rotation"]["z"]
    tf.transform.rotation.w = data["transform"]["rotation"]["w"]
    return tf


def get_desired_endoscope_pose_from_tag():
    ## Need to add time stamp and parent frame id
    desired_endo = TransformStamped()
    desired_endo.child_frame_id = "desired_endoscope"
    desired_endo.transform.translation.x = 0.0
    desired_endo.transform.translation.y = 0.0
    desired_endo.transform.translation.z = -0.1 # 10cm above the tag
    desired_endo.transform.rotation.x = 0.0
    desired_endo.transform.rotation.y = 0.0 
    desired_endo.transform.rotation.z = 0.0     
    desired_endo.transform.rotation.w = 1.0  # no rotation
    return desired_endo


######################### Rotation Representation Conversions #########################

def quaternion_rotate_vector(q, v):
    """
    Rotate a vector using a quaternion.
    
    Args:
        q (numpy.array): Quaternion in the form [x, y, z, w].
        v (numpy.array): Vector to rotate.
    
    Returns:
        numpy.array: Rotated vector.
    """
    vq = np.array([v[0], v[1], v[2], 0])
    q_inv = np.array([-q[0], -q[1], -q[2], q[3]])
    return quaternion_multiply(quaternion_multiply(q, vq), q_inv)[:3]


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    w3 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x3 = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y3 = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z3 = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([x3, y3, z3, w3])


def rotation_matrix_to_euler(rotation_matrix):
    """
    Convert a rotation matrix to (ZYX) Euler angles.

    Args:
    - rotation_matrix (numpy.ndarray): 3x3 rotation matrix.

    Returns:
    - roll (float): Rotation around the x-axis (in radians).
    - pitch (float): Rotation around the y-axis (in radians).
    - yaw (float): Rotation around the z-axis (in radians).
    """
    # Extract the Euler angles from the rotation matrix
    roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return roll, pitch, yaw


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert (ZYX) Euler angles to a rotation matrix.

    Args:
    - roll (float): Rotation around the x-axis (in radians).
    - pitch (float): Rotation around the y-axis (in radians).
    - yaw (float): Rotation around the z-axis (in radians).

    Returns:
    - rotation_matrix (numpy.ndarray): 3x3 rotation matrix.
    """
    # Calculate the trigonometric values
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    # Construct the rotation matrix
    rotation_matrix = np.array([
        [cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
        [sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
        [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll]
    ])

    return rotation_matrix


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    
    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.
        yaw (float): Yaw angle in radians.
    
    Returns:
        numpy.array: Quaternion in the form [x, y, z, w].
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x, y, z, w]


def quaternion_to_euler(quaternion):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        quaternion (numpy.array): Quaternion in the form [x, y, z, w].
    
    Returns:
        numpy.array: Euler angles in the form [roll, pitch, yaw].
    """
    # Extract quaternion components
    x, y, z, w = quaternion

    # Compute roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Compute pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Use ±π/2 if out of range
    else:
        pitch = np.arcsin(sinp)

    # Compute yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def getRotMtx(raw_pose):
    # take raw pose from the kinova and convert to rotation matrix
    # need to convert to radian
    R_0T = euler_to_rotation_matrix(raw_pose.theta_x*np.pi/180,
                                    raw_pose.theta_y*np.pi/180,
                                    raw_pose.theta_z*np.pi/180)
    return R_0T

def R2rot(R):
    """
    Recover k and theta from a 3 x 3 rotation matrix
    
        sin(theta) = | R-R^T |/2
        cos(theta) = (tr(R)-1)/2
        k = invhat(R-R^T)/(2*sin(theta))
        theta = atan2(sin(theta),cos(theta)
        
    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix    
    :rtype:  (numpy.array, number)
    :return: ( 3 x 1 k unit vector, rotation about k in radians)   

    from RPI Robotics Toolbox
    """
    
    R1 = R-R.transpose()
    
    sin_theta = np.linalg.norm(R1)/np.sqrt(8)
    
    cos_theta = (np.trace(R) - 1.0)/2.0
    theta = np.arctan2(sin_theta, cos_theta)
    
    #Avoid numerical singularity
    if sin_theta < 1e-6:
               
        if (cos_theta > 0):
            return [0,0,1], 0
        else:
            B = (1.0/2.0) *(R + np.eye(3))
            k = np.sqrt([B[0,0], B[1,1], B[2,2]])
            if np.abs(k[0]) > 1e-6:
                k[1] = k[1] * np.sign(B[0,1] / k[0])
                k[2] = k[2] * np.sign(B[0,2] / k[0])
            elif np.abs(k[1]) > 1e-6:
                k[2] = k[2] * np.sign(B[0,2] / k[1])
            return k, np.pi
    
    k = invhat(R1)/(2.0*sin_theta)    
    return k, theta


def invhat(khat):
    return np.array([(-khat[1,2] + khat[2,1]),(khat[0,2] - khat[2,0]),(-khat[0,1]+khat[1,0])])/2


def H_mtx_to_kinova_pose_in_base(H_mtx):
    # convert a homogeneous matrix to a kinova pose in the base frame
    # H_mtx is a 4x4 numpy array
    # returns a 6x1 numpy array [x,y,z,theta_x,theta_y,theta_z] in meters and degrees
    p = H_mtx[:3,3]
    R = H_mtx[:3,:3]
    roll, pitch, yaw = rotation_matrix_to_euler(R)
    return np.array([p[0], p[1], p[2], np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])


######################### Robot Command Functions #########################
class TCPArguments:
    def __init__(self):
        self.ip = "192.168.1.10"
        self.username = "admin"
        self.password = "admin"


def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def get_world_EE_HomoMtx(base):
    # get the homogeneous matrix of the end effector in the world frame
    current_pose = base.GetMeasuredCartesianPose()
    R = getRotMtx(current_pose)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([current_pose.x, current_pose.y, current_pose.z])

    return T


def get_joint_angles(base):
    # get the joint angles of the robot
    joints = base.GetMeasuredJointAngles()
    joint_angles = [joint.value for joint in joints.joint_angles]
    return joint_angles

def get_realtime_q_qdot(base_feedback):
    # will convert from degrees to radians
    joint_angles = [joint.value for joint in base_feedback.joint_angles]
    joint_velocities = [joint.value for joint in base_feedback.joint_velocities]
    return np.radians(joint_angles), np.radians(joint_velocities)

def move_to_home_position(base):
    TIMEOUT_DURATION = 10  # in seconds
    # Make sure the arm is in Single Level Servoing mode (high-level mode)
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to home position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("home position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def move_tool_pose_absolute(base, pose_kinova, speed=None):
    # move the end effector to an absolute position
    # pose_kinova is [x,y,z,theta_x,theta_y,theta_z] in meters and degrees

    print("Starting tool_pose absolute movement ...")
    action = Base_pb2.Action()
    action.name = "Tool Pose Movement"
    action.application_data = ""
    TIMEOUT_DURATION = 10  # in seconds
    if speed is not None:
        action.reach_pose.constraint.speed.translation = speed # m/s

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pose_kinova[0]    # (meters)
    cartesian_pose.y = pose_kinova[1]    # (meters)
    cartesian_pose.z = pose_kinova[2]    # (meters)
    cartesian_pose.theta_x = pose_kinova[3] # (degrees)
    cartesian_pose.theta_y = pose_kinova[4] # (degrees)
    cartesian_pose.theta_z = pose_kinova[5] # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("tool_pose absolute movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def move_tool_pose_relative(base, base_cyclic, pose_kinova, speed=None):
    # move the end effector to a relative position of the current pose
    # pose_kinova is [x,y,z,theta_x,theta_y,theta_z] in meters and degrees

    print("Starting tool_pose relative movement ...")
    action = Base_pb2.Action()
    action.name = "Tool Pose Relative Movement"
    action.application_data = ""
    TIMEOUT_DURATION = 10  # in seconds
    if speed is not None:
        action.reach_pose.constraint.speed.translation = speed

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + pose_kinova[0]    # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y + pose_kinova[1]    # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + pose_kinova[2]    # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + pose_kinova[3] # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + pose_kinova[4] # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + pose_kinova[5] # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("tool_pose relative movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def move_end_effector(base, desired_pose):
    ### function for Kinova Gen3 7 dof on Kortex API
    ### Fundation for vel control code, can correct both pos and ang error at the same time
    ### Author: Chuizheng Kong
    ### Created on: 05/09/2024
    
    # Make sure the arm is in Single Level Servoing mode (high-level mode)
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # init_pose = base.GetMeasuredCartesianPose()
    # get the desired rotation matrix
    R_d = getRotMtx(desired_pose)
    
    command = Base_pb2.TwistCommand()
    # note that twist is naturally in tool frame, but this conversion made things a bit easy
    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0.0  # adjust linear velocity
    twist.linear_y = 0.0
    twist.linear_z = 0.0
    twist.angular_x = 0.0
    twist.angular_y = 0.0
    twist.angular_z = 0.0  # adjust angular velocity

    max_exe_time = 10  # sec
    eps_pos = 0.001  # convergence criterion
    eps_ang = 0.01
    t_start = time.time()
    while True:
        current_pose = base.GetMeasuredCartesianPose()
        R = getRotMtx(current_pose)
        # reducing the pos error (in global frame!!)
        pos_diff = np.array([desired_pose.x-current_pose.x,
                                desired_pose.y-current_pose.y,
                                desired_pose.z-current_pose.z])
        
        # reducing ang error using ER = RRd^T
        ER = R @ R_d.T

        stopping, v, w = move_end_effector_vel(base, pos_diff, ER,
                                                max_vel=0.4, max_w=70.0, kp_pos=2.5, kp_ang=4.0, 
                                                eps_pos = eps_pos, eps_ang = eps_ang,
                                                t_start=t_start, max_exe_time=max_exe_time)
        if stopping:
            break
            
        twist.linear_x = v[0]
        twist.linear_y = v[1] 
        twist.linear_z = v[2]
        twist.angular_x = w[0]
        twist.angular_y = w[1]
        twist.angular_z = w[2]
        base.SendTwistCommand(command)
        #time.sleep(0.025)  # 40 Hz high-level control loop freq (ok might not need)
    return True


def move_end_effector_vel(base, pos_diff, ER, 
                          max_vel=0.4, max_w=70.0, kp_pos=2.5, kp_ang=4.0, 
                          eps_pos = 0.001, eps_ang = 0.01,
                          dcc_factor = 2, ang_dcc_factor = 6,
                          t_start=None, max_exe_time=10):
    ### function for Kinova Gen3 7 dof on Kortex API
    ### takes the error in world (BASE) frame of the robot arm
    ### Fundation for vel control code, can correct both pos and ang error at the same time


    dcc_range = max_vel / (kp_pos * dcc_factor)  # dcc_range should be smaller than max_vel/kp_pos
    ang_dcc_range = max_w / (kp_ang * ang_dcc_factor)  # dcc_range should be smaller than max_vel/kp_pos

    pos_diff_norm = np.linalg.norm(pos_diff)+1e-5
    v_temp = max_vel * pos_diff/pos_diff_norm

    # frobenius norm of matrix squre root of ER or eR2
    k,theta = R2rot(ER)
    k=np.array(k)
    eR2=-np.sin(theta/2)*k * 180 / np.pi  # not the best name but works fine
    
    eR2_norm = np.linalg.norm(eR2)+1e-5
    w_temp = max_w * eR2/eR2_norm

    # print(pos_diff_norm, eR2_norm)

    reached = pos_diff_norm < eps_pos and eR2_norm < eps_ang
    timeout = t_start is not None and time.time() > t_start + max_exe_time
    stopping = reached or timeout
    
    if stopping:
        print("Robot Stop")
        if reached:
            print("Goal Pose reached")
        base.Stop()
        
    # go in max vel when outside dcc_range
    if pos_diff_norm < dcc_range:
        v = kp_pos * pos_diff
    else:
        v = v_temp

    if eR2_norm < ang_dcc_range:
        kR = kp_ang*np.eye(3)
        w = kR @ eR2 
    else:
        w = w_temp
    
    return stopping, v, w


def move_joints(base, desired_joints):
    # move the joints to a desired position
    # desired_joints is a list of joint angles in degrees
    # joint 1 is the base joint, joint 7 is the end effector joint
    print("Starting joint movement ...")
    action = Base_pb2.Action()
    action.name = "Joint Movement"
    action.application_data = ""
    TIMEOUT_DURATION = 10  # in seconds

    for joint_id in range(7):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = desired_joints[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("joint movement completed")
    else:
        print("Timeout on action notification wait")
    return finished