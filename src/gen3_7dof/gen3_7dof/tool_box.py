from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

import numpy as np

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


def example_move_to_home_position(base):
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


def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles to a rotation matrix.

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


def move_end_effector(base, desired_pose):
    ### function for Kinova Gen3 7 dof on Kortex API
    ### Fundation for vel control code, can correct both pos and ang error at the same time
    ### Author: Chuizheng Kong
    ### Created on: 05/09/2024
    
    max_vel = 0.5-0.1  # m/s  0.5 is max
    #max_vel = 0.1
    max_w = 70.0  # ~ 50 deg/s
    kp_pos = 2.5
    kp_ang = 4.0

    dcc_range = max_vel / (kp_pos * 2)  # dcc_range should be smaller than max_vel/kp_pos
    ang_dcc_range = max_w / (kp_ang * 6)
    
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
        
        pos_diff_norm = np.linalg.norm(pos_diff)
        v_temp = max_vel * pos_diff/pos_diff_norm

        # reducing ang error using ER = RRd^T
        ER = R @ R_d.T
        # frobenius norm of matrix squre root of ER or eR2
        k,theta = R2rot(ER)
        k=np.array(k)
        eR2=-np.sin(theta/2)*k * 180 / np.pi  # not the best name but works fine
        
        eR2_norm = np.linalg.norm(eR2)+1e-5
        w_temp = max_w * eR2/eR2_norm

        # print(pos_diff_norm, eR2_norm)

        reached = pos_diff_norm < eps_pos and eR2_norm < eps_ang
        
        if reached or time.time()>t_start+max_exe_time:
            print("Robot Stop")
            if reached:
                print("Goal Pose reached")
            base.Stop()
            break
            
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
            
        twist.linear_x = v[0]
        twist.linear_y = v[1] 
        twist.linear_z = v[2]
        twist.angular_x = w[0]
        twist.angular_y = w[1]
        twist.angular_z = w[2]
        base.SendTwistCommand(command)
        #time.sleep(0.025)  # 40 Hz high-level control loop freq (ok might not need)
    return True