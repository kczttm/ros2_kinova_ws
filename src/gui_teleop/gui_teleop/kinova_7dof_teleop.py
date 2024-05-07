#!/usr/bin/python3

# GUI for Kinova Gen3 7dof robot arm
# Self/Workspace collision check not implemented, drive with caution 
import sys, os, time, argparse, traceback
from tkinter import *
from tkinter import messagebox
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from controller_manager_msgs.srv import ListControllers, SwitchController

class GUI_Teleop(Node):
    def __init__(self):
        super().__init__('gui_teleop_7dof')

        cust_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        self.sub = self.create_subscription(JointState, 
                                            '/joint_states',
                                            self.update_label,
                                            qos_profile=cust_qos_profile)
        self.pub = self.create_publisher(Twist, 
                                         'twist_controller/commands', 
                                        10)
        self.sub
        

        self.gui = Tk()
        self.gui.title('Kinova 7DoF')
        self.jobid = None
        
        self.gui.title = "Robot State"
        self.speed = Scale(orient='vertical', 
                           label='speed control',
                           from_=0.1, 
                           length=500,
                           resolution=0.1,
                           to=5.)
        self.speed.set(1.)
        
        self.speed.pack(side=RIGHT)

        self.ctrl_list = Label(self.gui, fg='black', justify=LEFT)
        self.ctrl_list.pack()

        self.active_controllers = ['joint_trajectory_controller']

        #switch_ctrl=Button(self.gui,text='switch controller')

        self.label = Label(self.gui, fg='black',justify=LEFT)
        self.label.pack()
        # self.label.after(250, self.update_label)
        
        left=Button(self.gui,text='y+')
        right=Button(self.gui,text='y-')
        forward=Button(self.gui,text='x+')
        backward=Button(self.gui,text='x-')
        up=Button(self.gui,text='z+')
        down=Button(self.gui,text='z-')

        Rx_n=Button(self.gui,text='Rx_n')
        Rx_p=Button(self.gui,text='Rx_p')
        Ry_n=Button(self.gui,text='Ry_n')
        Ry_p=Button(self.gui,text='Ry_p')
        Rz_n=Button(self.gui,text='Rz_n')
        Rz_p=Button(self.gui,text='Rz_p')

        # j1_n=Button(self.gui,text='j1_n')
        # j1_p=Button(self.gui,text='j1_p')
        # j2_n=Button(self.gui,text='j2_n')
        # j2_p=Button(self.gui,text='j2_p')
        # j3_n=Button(self.gui,text='j3_n')
        # j3_p=Button(self.gui,text='j3_p')
        # j4_n=Button(self.gui,text='j4_n')
        # j4_p=Button(self.gui,text='j4_p')
        # j5_n=Button(self.gui,text='j5_n')
        # j5_p=Button(self.gui,text='j5_p')
        # j6_n=Button(self.gui,text='j6_n')
        # j6_p=Button(self.gui,text='j6_p')



        # gripper=Button(self.gui,text='gripper off',command=lambda: gripper_ctrl(tool),bg='red')

        v_des = .02 # m/s
        w_des = 1 # somehow looks like degree /s ???
        #switch_ctrl.bind('<ButtonPress-1>', lambda event: self.switch_controller_twist())

        left.bind('<ButtonPress-1>', lambda event: self.move([0,v_des,0],[0.,0.,0.,]))
        right.bind('<ButtonPress-1>', lambda event: self.move([0,-v_des,0],[0.,0.,0.,]))
        forward.bind('<ButtonPress-1>', lambda event: self.move([v_des,0,0],[0.,0.,0.,]))
        backward.bind('<ButtonPress-1>', lambda event: self.move([-v_des,0,0],[0.,0.,0.,]))
        up.bind('<ButtonPress-1>', lambda event: self.move([0,0,v_des],[0.,0.,0.,]))
        down.bind('<ButtonPress-1>', lambda event: self.move([0,0,-v_des],[0.,0.,0.,]))

        Rx_n.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[w_des,0,0]))
        Rx_p.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[-w_des,0,0]))
        Ry_n.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[0,w_des,0]))
        Ry_p.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[0,-w_des,0]))
        Rz_n.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[0,0,w_des]))
        Rz_p.bind('<ButtonPress-1>', lambda event: self.move([0.,0.,0.],[0,0,-w_des]))

        # j1_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([-0.1,0.,0.,0.,0.,0.])))
        # j1_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([+0.1,0.,0.,0.,0.,0.])))
        # j2_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,-0.1,0.,0.,0.,0.])))
        # j2_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,+0.1,0.,0.,0.,0.])))
        # j3_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,-0.1,0.,0.,0.])))
        # j3_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,+0.1,0.,0.,0.])))
        # j4_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,-0.1,0.,0.])))
        # j4_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,+0.1,0.,0.])))
        # j5_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,0.,-0.1,0.])))
        # j5_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,0.,+0.1,0.])))
        # j6_n.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,0.,0.,-0.1])))
        # j6_p.bind('<ButtonPress-1>', lambda event: self.movej(np.array([0.,0.,0.,0.,0.,+0.1])))


        left.bind('<ButtonRelease-1>', lambda event: self.stop())
        right.bind('<ButtonRelease-1>', lambda event: self.stop())
        forward.bind('<ButtonRelease-1>', lambda event: self.stop())
        backward.bind('<ButtonRelease-1>', lambda event: self.stop())
        up.bind('<ButtonRelease-1>', lambda event: self.stop())
        down.bind('<ButtonRelease-1>', lambda event: self.stop())

        Rx_n.bind('<ButtonRelease-1>', lambda event: self.stop())
        Rx_p.bind('<ButtonRelease-1>', lambda event: self.stop())
        Ry_n.bind('<ButtonRelease-1>', lambda event: self.stop())
        Ry_p.bind('<ButtonRelease-1>', lambda event: self.stop())
        Rz_n.bind('<ButtonRelease-1>', lambda event: self.stop())
        Rz_p.bind('<ButtonRelease-1>', lambda event: self.stop())

        # j1_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j1_p.bind('<buttonrelease-1>', lambda event: self.stop())
        # j2_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j2_p.bind('<buttonrelease-1>', lambda event: self.stop())
        # j3_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j3_p.bind('<buttonrelease-1>', lambda event: self.stop())
        # j4_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j4_p.bind('<buttonrelease-1>', lambda event: self.stop())
        # j5_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j5_p.bind('<buttonrelease-1>', lambda event: self.stop())
        # j6_n.bind('<buttonrelease-1>', lambda event: self.stop())
        # j6_p.bind('<buttonrelease-1>', lambda event: self.stop())



        # gripper.pack()

        #switch_ctrl.pack()

        left.pack(in_=self.gui, side=LEFT)
        right.pack(in_=self.gui, side=RIGHT)
        forward.pack(in_=self.gui, side=LEFT)
        backward.pack(in_=self.gui, side=RIGHT)
        up.pack(in_=self.gui, side=LEFT)
        down.pack(in_=self.gui, side=RIGHT)

        Rx_n.pack()
        Rx_p.pack()
        Ry_n.pack()
        Ry_p.pack()
        Rz_n.pack()
        Rz_p.pack()

        # j1_n.pack(in_=self.gui, side=LEFT)
        # j1_p.pack(in_=self.gui, side=LEFT)
        # j2_n.pack(in_=self.gui, side=LEFT)
        # j2_p.pack(in_=self.gui, side=LEFT)
        # j3_n.pack(in_=self.gui, side=LEFT)
        # j3_p.pack(in_=self.gui, side=LEFT)
        # j4_n.pack(in_=self.gui, side=LEFT)
        # j4_p.pack(in_=self.gui, side=LEFT)
        # j5_n.pack(in_=self.gui, side=LEFT)
        # j5_p.pack(in_=self.gui, side=LEFT)
        # j6_n.pack(in_=self.gui, side=LEFT)
        # j6_p.pack(in_=self.gui, side=LEFT)

        self.update_active_controllers()
        if not 'twist_controller' in self.active_controllers:
            self.switch_controller_twist()


    def update_active_controllers(self):
        list_controllers_client = self.create_client(ListControllers, 
                                                   '/controller_manager/list_controllers')
        
        while not list_controllers_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /controller_manager/list_controllers not available, waiting...')
        
        request = ListControllers.Request()
        future = list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.active_controllers = []
            controller_list = "Active controllers:\n"
            for controller in future.result().controller:
                if controller.state == "active":
                    controller_list += " - %s\n" % controller.name
                    if controller.name != 'joint_state_broadcaster':
                        self.active_controllers.append(controller.name)
        else:
            controller_list = "Failed to obtain controllers"
        self.ctrl_list.config(text = controller_list)
    

    def switch_controller_twist(self, controller_name='twist_controller'):
        switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /controller_manager/switch_controller not available, waiting...')

        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.deactivate_controllers = self.active_controllers
        request.strictness = 1  # Strictness level: BEST_EFFORT - 2
        request.activate_asap = True
        future = switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(future.result())
            self.update_active_controllers()  # Update the list of active controllers
            return True
        else:
            self.get_logger().error(f'Failed to switch to controller: {controller_name}')
            return False


    def update_label(self,msg):
        joint_pos = "Robot Joint Positions (deg):\n\n"
        for j in msg.position:
            joint_pos += "%.2f\n" % np.rad2deg(j)

        self.label.config(text = joint_pos)
        return 

    def move(self, vd, wd):
        try:
            twist = Twist()
            twist.linear.x = vd[0]*self.speed.get()
            twist.linear.y = vd[1]*self.speed.get()
            twist.linear.z = vd[2]*self.speed.get()
            twist.angular.x = wd[0]*self.speed.get()
            twist.angular.y = wd[1]*self.speed.get()
            twist.angular.z = wd[2]*self.speed.get()
            self.pub.publish(twist)

            self.jobid = self.gui.after(10, lambda: self.move(vd,wd))
        except:
            traceback.print_exc()
        return
    
    def stop(self):
        self.gui.after_cancel(self.jobid)
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return


def main(args=None):
    rclpy.init(args=args)
    kinova_gui = GUI_Teleop()

    process_thread = threading.Thread(target=rclpy.spin, args=(kinova_gui,))
    process_thread.start()

    kinova_gui.gui.mainloop()

    kinova_gui.destroy_node()
    rclpy.shutdown()
    process_thread.join()


if __name__ == '__main__':
    main()