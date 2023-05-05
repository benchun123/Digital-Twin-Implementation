# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
import os
import argparse
import time

import carb
import omni
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.robots import Robot
from isaac_controller import uArmController

import numpy as np

# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")
# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
simulation_app.update()
result, check = omni.kit.commands.execute("RosBridgeRosMasterCheck")
if not check:
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

# Tick all of the components once to make sure all of the ROS nodes are initialized
# For cameras this also handles viewport initialization etc.
# omni.kit.commands.execute("Ros2BridgeUseSimTime",
#                           use_sim_time=True)
omni.kit.commands.execute("RosBridgeUseSimTime", use_sim_time=True)

# tick_rate_hz = 20.0
# time_dt = 1.0 / tick_rate_hz
# print(f"Running sim at {tick_rate_hz} Hz, with dt of {time_dt}")
# simulation_context = SimulationContext(
#     physics_dt=time_dt, rendering_dt=time_dt)
# omni.kit.commands.execute("RosBridgeUseSimTime", use_sim_time=True)

# Note that this is not the system level rospy, but one compiled for omniverse
import numpy as np
import rospy
from std_msgs.msg import Empty
import time

from time import sleep
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from uarm_msgs.msg import uarm_state
from sensor_msgs.msg import JointState
import threading
# uarm_state
# bool ready
# bool pump
# int16 x
# int16 y
# int16 z
# int16 w

base_folder = "/home/benchun/benchun/IsaacProjects/Demonstrator/"
uarm_usd_path = base_folder + "src/uarm/urdf/light_model/light_model.usd"

class uArm_Isaac_Interface():
    def __init__(self, asset_path=uarm_usd_path, ns ="", part=""):
        # setting up the world
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=0.01)
        # self.ros_world.scene.add_default_ground_plane() // need accoun login
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/uarm")
        self.robot = self.ros_world.scene.add(Robot(prim_path="/World/uarm", name="uarm_n", position=np.array([0, 0, 0])))
        self.ros_world.reset()

        #functional parameters
        self.ns = ns
        self.part = part
        self.state = uarm_state()
        self.busy = False
        self.pump_state = False
        self.joints = [0]*5
        self.joint_state = [0]*5
        self.joint_state_pos = [-1]*5
        self.joint_names = ['Joint1','Joint2','Joint3','Joint8','JointTCP']
        #lengths and angles
        self.l1 = 0.0132*1000 #delta x - joint1 to joint2
        self.l2 = 0.14207*1000 #delta z - joint2 to joint3
        self.l3 = 0.15852*1000 #delta x - joint3 to joint8
        self.l4 = 0.0545*1000 #delta x - joint8 to pump nozzle
        self.l5 = 0.1056*1000 #delta z joint 2 to ground
        self.l6 = 0.0745*1000 #delta z - joint8 to pump nozzle
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.pos_w = 0
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.target_w = 0

        self.pub1 = rospy.Publisher(str(self.ns+'joint1/command'), Float64, queue_size=1)
        self.pub2 = rospy.Publisher(str(self.ns+'joint2/command'), Float64, queue_size=1)
        self.pub3 = rospy.Publisher(str(self.ns+'joint3/command'), Float64, queue_size=1)
        self.pub8 = rospy.Publisher(str(self.ns+'joint8/command'), Float64, queue_size=1)
        self.pubtcp = rospy.Publisher(str(self.ns+'jointtcp/command'), Float64, queue_size=1)
       
        # publish automatically or callback ...
        # self.publisher = threading.Thread(target=self.pub_state)
        # self.publisher.start()
        self.lis = rospy.Subscriber(str(self.ns+"joint_states"), JointState, self.pub_state_callback)
        self.pub_busy = rospy.Publisher(str(self.ns+"busy"), Bool, queue_size=1)
        # self.pub_pos = rospy.Publisher(str(self.ns+"position"), Float64, queue_size=1) # not just one float

        # reset some param
        rospy.loginfo("Num of degrees of freedom after first reset: " + str(self.robot.num_dof))
        self.controller = uArmController()
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " interface initialized")


    def pub_state_callback(self, data):
        if self.joint_state_pos[0] < 0:
            self.get_joint_state_pos(data.name)
        i = 0
        while i < len(self.joint_state):
            self.joint_state[i] = data.position[self.joint_state_pos[i]]
            i=i+1
        self.check_position(self.target_x, self.target_y, self.target_z, self.target_w)
        self.pub_cart(self.joint_state[0], self.joint_state[1], self.joint_state[2],self.joint_state[4])
        self.pub_busy.publish(self.busy)
        self.pub_joints(self.joints)

    def get_joint_state_pos(self, label):
        i = 0
        k = 0
        for j in self.joint_names:
            while self.joint_state_pos[k] < 0 and i < 1000:
                if label[i] == j:
                    self.joint_state_pos[k] = i
                else:
                    i = i+1
            if i == 1000:
                print("didnt find Joint")
            k = k+1

    def check_position(self, x, y, z, w):
        if x+2 >= self.pos_x and x-2 <= self.pos_x and y-2 <= self.pos_y and y+2 >= self.pos_y and z-4 <= self.pos_z and z+2 >= self.pos_z and w-2 <= self.pos_w and w+2 >= self.pos_w:
            self.busy = False
            return True
        else:       
            self.busy = True 
            return False

    def pub_cart(self,theta,alpha,beta,epsilon):
        r = self.l1+self.l2*np.sin(alpha)+self.l3*np.cos(alpha+beta)+self.l4
        self.point = uarm_state()
        self.point.x = int(np.around(r*np.cos(theta)))
        self.point.y = int(np.around(r*np.sin(theta)))
        self.point.z = int(np.around(self.l5+self.l2*np.cos(alpha)-self.l3*np.sin(alpha+beta)-self.l6))
        self.point.w = int(epsilon/3.14159*180)
        self.point.pump = self.pump_state
        if self.busy:
            self.ready = False
        else:
            self.ready = True
        self.pos_x = self.point.x
        self.pos_y = self.point.y
        self.pos_z = self.point.z
        self.pos_w = self.point.w
        self.pub_pos.publish(self.point)

    def pub_joints(self, joints):
        self.pub1.publish(joints[0])
        self.pub2.publish(joints[1])
        self.pub3.publish(joints[2])
        self.pub8.publish(-self.joint_state[2]-self.joint_state[1])
        self.pubtcp.publish(joints[4])
        rospy.loginfo('joints: [%f, %f, %f, %f, %f]' % (joints[0], joints[1], joints[2], joints[3], joints[4]))
        rospy.loginfo('joints pub: [%f, %f, %f, %f, %f]' % (joints[0], joints[1], joints[2], 
                        -self.joint_state[2]-self.joint_state[1], joints[4]))
        
    def read(self):
        # TODO: find if we need to set these parameters
        # joint_pos = self.robot.get_joint_positions()
        # self.pos = joint_pos
        rospy.loginfo("read slider info ")
        return True

    def check_limits(self,x,y,z,w):
        lim_x = [30,280]
        lim_y = [-278,278]
        lim_z = [-120, 130]
        lim_w = [0,180]
        lim_r = [150,280]
        limits = [lim_x,lim_y,lim_z,lim_w,lim_r]
        r = (x**2+y**2)**0.5
        coordinates = [x,y,z,w,r]
        for i in [0,1,2,3,4]:
            if coordinates[i] < limits[i][0] or coordinates[i] > limits[i][1]:
                rospy.loginfo('%s: target position out of range [%i, %i, %i, %i]' % (str(self.ns+self.part), x, y, z, w))
                return False
        return True
    
    def calc_joints(self, x, y, z, w):
        #angles
        theta = 0 #joint1 angle (rot z axis)
        alpha = 0 #joint2 angle (rot y axis)
        beta = 0 #joint3 angle (rot y axis)
        gamma = 0 #joint8 angle (rot y axis)
        delta = 0 #angle at joint 2 of the links triangle
        sigma = 0 #angle between c and x-y plane
        epsilon = 0 #tcp angle

        #calculate lengths of the triangle of the links

        r = (x**2+y**2)**0.5
        a = self.l2
        b = self.l3
        c = ((r-self.l1-self.l4)**2+(z+self.l6-self.l5)**2)**0.5

        #calculate angles
        if y == 0:
            theta = 0
        else:
            theta = np.arcsin(y/r)
        delta= np.arccos(-(b**2-a**2-c**2)/(2*a*c))
        beta = np.pi/2-np.arccos(-(c**2-a**2-b**2)/(2*a*b))
        sigma = np.arcsin((z+self.l6-self.l5)/c)
        alpha = np.pi/2-delta-sigma     
        gamma = -beta-alpha
        epsilon = w/180*3.14159

        self.joints = [theta, alpha, beta, gamma, epsilon]
        print("joints ", self.joints)
        return self.joints 

    # TODO: how to move
    def move(self, x, y, z, w=90):
        rospy.loginfo('%s: new goal received [%i, %i, %i, %i]' % (str(self.ns+self.part), x, y, z, w))

        if not self.check_limits(x, y, z, w):
            return False
        self.joints = self.calc_joints(x, y, z, w)

        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_w = w
        self.busy = True
        self.pub_joints(self.joints)
        return True

    # TODO: how to move
    def auto_move(self, x, y, z, w):
        rospy.loginfo("auto move function")

    # TODO: how to pump
    def pump(self, data):
        rospy.loginfo("pump function")

    # TODO: how to reset?
    def reset(self):
        self.joints = [0]*5
        rospy.loginfo(rospy.get_caller_id() + " stopping slider")
        return True


    def pub_state(self):
        while not rospy.is_shutdown():
            # move the animation to run_simulation
            # if self.write_msg:
            #     self.ser.write(self.msg.encode())
            #     self.write_msg = False
            #     self.msg = "Slider: "
            #     self.write_move = False
            #     self.write_shift = False
            #     self.write_timeout = False
            #     self.write_stop = False
            self.read()
            self.pub_busy.publish(self.busy)
            # self.pub_pos.publish(self.pos)
            # self.pub_slider_state.publish(self.state)
            sleep(1)

    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 0:
                    self.ros_world.reset()

                # # # the actual setting the cube pose is done here
                # self.robot.apply_action(self.controller.forward(command=[self.state.target_raw]))
                self.robot.apply_action(self.controller.forward(command=self.joints))
                # target_pos = np.array([self.state.pos_raw])
                # self.robot.set_joint_positions(target_pos)
                # rospy.loginfo("read slider info ")
                # self.joint_position_np = np.array([np.random.rand()*3, np.random.rand()*3, 0, np.random.rand()*3, np.random.rand()*3])
                # self.robot.set_joint_positions(self.joint_position_np)

        # Cleanup
        rospy.signal_shutdown("uArm_Isaac_Interface_Node ends")
        self.timeline.stop()
        simulation_app.close()
    
if __name__ == '__main__':
    # # Start up the simulator
    # simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
    # # enable ROS bridge extension
    # from omni.isaac.core.utils.extensions import enable_extension
    # enable_extension("omni.isaac.ros_bridge")
    
    
    rospy.init_node("uArm_Isaac_Interface_Node", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    # Load the stage
    slider_ns="uarm/"
    scene_n = uArm_Isaac_Interface(ns=slider_ns)
    scene_n.run_simulation()

