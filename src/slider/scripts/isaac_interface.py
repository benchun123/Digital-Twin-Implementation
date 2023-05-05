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
from isaac_controller import SliderController
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
# import numpy as np
# import time

# import rospy
# from std_msgs.msg import Empty
# from std_msgs.msg import Float64
# from control_msgs.msg import JointControllerState
# from std_msgs.msg import Bool

# from uarm_msgs.srv import *
# from std_srvs.srv import *
# import uarm_msgs.msg
# import json
# import actionlib
# from scripts.real_interface import Slider_sim_Server

from time import sleep
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from uarm_msgs.msg import slider_state
from std_srvs.srv import *
import threading

#################################
#                               #
#       valid positions         #
#                               #
#      y element [-470, 0]      #
#                               #
#################################

base_folder = "/home/benchun/benchun/IsaacProjects/Demonstrator/"
slider_usd_path = base_folder + "src/slider/urdf/model/model.usd"

class Slider_Isaac_Interface():
    def __init__(self, asset_path=slider_usd_path, ns = "", target_tolerance = 2):
        # setting up the world
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=0.01)
        self.ros_world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/slider")
        self.robot = self.ros_world.scene.add(Robot(prim_path="/World/slider", name="slider_n", position=np.array([0, 0, 0])))
        self.ros_world.reset()

        #handle communication // TODO: will remove later
        self.write_msg = False
        self.msg = "Slider: "
        self.write_move = False
        self.write_shift = False
        self.write_stop = False  
        self.write_to_limit = False
        self.to_limit = 0    

        #functional parameters
        self.ns = ns
        self.state = slider_state()
        self.lim = [-470,0]
        self.target_tolerance = target_tolerance
        self.pos = None
        self.state.pos_raw = None
        self.state.target_raw = None
        self.busy = False
        self.state.shift = 40
        self.state.timeout = 20
        self.state.limit_switch = None
        self.state.drive_to_limit = None
        
        self.pub_busy = rospy.Publisher(str(self.ns+"busy"), Bool, queue_size=1)
        self.pub_pos = rospy.Publisher(str(self.ns+"position"), Float64, queue_size=1)
        self.pub_slider_state = rospy.Publisher(str(self.ns+"state_raw"), slider_state, queue_size=1)

        self.publisher = threading.Thread(target=self.pub_state)
        self.publisher.start()

        # reset some param
        rospy.loginfo("Num of degrees of freedom after first reset: " + str(self.robot.num_dof))
        pos_default = np.zeros(self.robot.num_dof, dtype="float64") 
        self.pos = pos_default
        self.state.pos_raw = int(pos_default)
        self.state.target_raw = int(pos_default)
        self.state.limit_switch = int(0)
        self.state.drive_to_limit = int(0)

        self.controller = SliderController()
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " interface initialized")


    def read(self):
        # TODO: find if we need to set these parameters
        joint_pos = self.robot.get_joint_positions()
        self.pos = joint_pos
        self.state.pos_raw = int(joint_pos)
        self.state.target_raw = self.state.target_raw
        # self.state.shift = self.state.shift
        # self.state.timeout = self.state.timeout
        # self.state.limit_switch = self.state.limit_switch
        # self.state.drive_to_limit = self.state.drive_to_limit
        # self.busy = self.busy
        # rospy.loginfo("read slider info ")
        return True

    def move(self, goal):
        sleep(.5)
        if not self.write_move and not self.write_stop:
            if not self.check_limits(int(goal)) or self.busy:
                rospy.loginfo(rospy.get_caller_id() + " target invalid - rejecting move command")
                return False
            else:
                rospy.loginfo(rospy.get_caller_id() + " slider recieved move command to %i", goal) 
                # goal = int(470+goal)
                self.state.target_raw = int(goal/10.0)
                return True
        else:
            rospy.loginfo(rospy.get_caller_id() + " slider busy - rejecting move command")
            return False

    def check_limits(self, y):
        if y < self.lim[0] or y > self.lim[1]:
            return False
        else:
            return True

    # TODO: how to stop the slider?
    def stop(self):
        self.msg = "Slider: Position:900 "
        self.write_stop = True
        self.write_msg = True
        self.busy = False
        rospy.loginfo(rospy.get_caller_id() + " stopping slider")
        return True


    #set position shift from echo sensor in mm - recommended: 100 to *TO BE EVALUATED!*
    def set_shift(self, data = 100):
        self.state.shift = data
        rospy.loginfo(rospy.get_caller_id() + " setting slider's position shift to %i", data)
        return True

    def set_timeout(self, data = 20):
        self.state.timeout = data
        rospy.loginfo(rospy.get_caller_id() + " setting slider's timeout to %i", data)
        return True

    # TODO: what is the function of drive to limie
    def set_drive_to_limit(self):
        self.state.target_raw = self.lim[0]/10
        rospy.loginfo(rospy.get_caller_id() + " activating sliders drive to limit function")
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
            self.pub_pos.publish(self.pos)
            self.pub_slider_state.publish(self.state)
            sleep(1)

    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            if self.ros_world.is_playing():
                if self.ros_world.current_time_step_index == 0:
                    self.ros_world.reset()

                # # # the actual setting the cube pose is done here
                # self.state.target_raw = self.state.target_raw
                self.robot.apply_action(self.controller.forward(command=[self.state.target_raw]))
                # target_pos = np.array([self.state.pos_raw])
                # self.robot.set_joint_positions(target_pos)
                # rospy.loginfo("read slider info ")

        # Cleanup
        rospy.signal_shutdown("Slider_Isaac_Interface_Node ends")
        self.timeline.stop()
        simulation_app.close()
    
if __name__ == '__main__':
    # # Start up the simulator
    # simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
    # # enable ROS bridge extension
    # from omni.isaac.core.utils.extensions import enable_extension
    # enable_extension("omni.isaac.ros_bridge")
    
    
    rospy.init_node("Slider_Isaac_Interface_Node", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    # Load the stage
    slider_ns="slider/"
    scene_n = Slider_Isaac_Interface(ns=slider_ns)
    # scene_n.run_simulation()

