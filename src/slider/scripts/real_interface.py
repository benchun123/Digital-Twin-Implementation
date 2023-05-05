#!/usr/bin/env python

import serial
from time import sleep
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from uarm_msgs.msg import slider_state
from std_srvs.srv import *
import threading

class Slider_real_interface:

    def __init__(self, ns = "", target_tolerance = 2):

        self.ns = ns

        #serial connection
        self.failed_reading_counter = 0
        self.COM = None
        self.BAUD = 115200
        self.ser = None
        self.find_slider()

        #handle communication
        self.write_msg = False
        self.msg = "Slider: "
        self.write_move = False
        self.write_shift = False
        self.write_stop = False  
        self.write_to_limit = False
        self.to_limit = 0    

        #functional parameters

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

        self.read()
        sleep(1)
        self.read()
        sleep(1)
        self.read()
        sleep(1)

        self.publisher = threading.Thread(target=self.pub_state)
        self.publisher.start()

        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " interface initialized")

    def find_slider(self):

        #find slider's port
        rospy.loginfo(rospy.get_caller_id() + " " + self.ns + " searching slider")
        for COM in ['/dev/ttyACM0', '/dev/ttyACM1','/dev/ttyACM2', '/dev/ttyACM3','/dev/ttyACM4', '/dev/ttyACM5','/dev/ttyACM6', '/dev/ttyACM7','/dev/ttyACM8', '/dev/ttyACM9']:
            try:
                ser = serial.Serial(COM, self.BAUD, timeout = .1)
                           
                for i in range(10):
                    val = str(ser.readline().decode().strip('\r\n'))            
                    if val:
                        for j in range(15):
                            val = str(ser.readline().decode().strip('\r\n'))
                            if val.startswith("Slider"):
                                rospy.loginfo(rospy.get_caller_id() + " Slider identified on port " + str(COM))
                                self.COM = COM
                                self.ser = serial.Serial(self.COM, self.BAUD, timeout = .1)
                                self.failed_reading_counter = 0
                                return
                            sleep(.1)
                    sleep(.1)

            except:
                rospy.loginfo(rospy.get_caller_id() + " no device found at " + COM)

    def read(self):
        val = "reading error"
        try:
            self.ser.reset_input_buffer()
            val = str(self.ser.readline().decode().strip('\r\n'))#Capture serial output as a decoded string
        except:
            rospy.loginfo(rospy.get_caller_id() + " failed to read serial")
            self.failed_reading_counter += 1
            if self.failed_reading_counter > 5:
                self.find_slider()
            return

        self.failed_reading_counter = 0
        #rospy.loginfo(rospy.get_caller_id() + " recieved from serial: " + val)

        if val.startswith("Slider"):

            #extract values from slider's serial message
            val_split = val.split()
            #print(val_split)
            self.pos = float(int(val_split[1].split(":")[1])-470)
            self.state.pos_raw = int(int(val_split[1].split(":")[1]))
            self.state.shift = int(val_split[3].split(":")[1])
            self.state.target_raw = int(val_split[4].split(":")[1])
            self.state.timeout = int(val_split[5].split(":")[1])
            self.state.limit_switch = int(val_split[6].split(":")[1])
            self.state.drive_to_limit = int(val_split[7].split(":")[1])
            if val_split[2].split(":")[1] == "false":
                self.busy = True
            else:
                self.busy = False

    def move(self, goal):
        #print(goal)
        sleep(.5)
        if not self.write_move and not self.write_stop:
            if not self.check_limits(int(goal)) or self.busy:
                rospy.loginfo(rospy.get_caller_id() + " target invalid - rejecting move command")
                return False
            else:
                rospy.loginfo(rospy.get_caller_id() + " slider recieved move command to %i", goal) 
                goal = int(470+goal)
                self.msg += str("Slider: Position:" + str(goal) + " ")                     
                self.busy = True
                self.write_move = True
                self.write_msg = True           
                return True
        else:
            rospy.loginfo(rospy.get_caller_id() + " slider busy - rejecting move command")
            return False
        '''
        if not self.check_limits(int(goal)) or self.busy:
            return False
        else:
            goal = int(470+goal)
            msg = str("Slider: Position:" + str(goal)+" ")
            #print(msg)
            self.ser.write(msg.encode())
            self.busy = True
            sleep(.5)            
            #print("goal sent")
            return True       
        '''

    def check_limits(self, y):
        if y < self.lim[0] or y > self.lim[1]:
            return False
        else:
            return True

    def stop(self):
        #self.goal = self.pos
        self.msg = "Slider: Position:900 "
        self.write_stop = True
        self.write_msg = True
        self.busy = False
        rospy.loginfo(rospy.get_caller_id() + " stopping slider")
        return True
        '''
        try:
            self.ser.write("Slider: Position:900 ".encode())
            rospy.loginfo(rospy.get_caller_id() + " stopped slider")
            return True
        except:
            return False
        '''

    #set position shift from echo sensor in mm - recommended: 100 to *TO BE EVALUATED!*
    def set_shift(self, data = 100):
        #self.goal = self.pos
        self.msg += "Slider: Shift:" + str(data) + " "
        self.write_shift = True
        self.write_msg = True
        rospy.loginfo(rospy.get_caller_id() + " setting slider's position shift to %i", data)
        return True

    def set_timeout(self, data = 20):
        #self.goal = self.pos
        self.msg += "Slider: Timeout:" + str(data) + " "
        self.write_timeout = True
        self.write_msg = True
        rospy.loginfo(rospy.get_caller_id() + " setting slider's timeout to %i", data)
        return True

    def set_drive_to_limit(self):
        #self.goal = self.pos
        self.msg += "Slider: To_Limit:1 "
        self.write_to_Limit = True
        self.write_msg = True
        rospy.loginfo(rospy.get_caller_id() + " activating sliders drive to limit function")
        return True

    def pub_state(self):

        while not rospy.is_shutdown():
            if self.write_msg:
                self.ser.write(self.msg.encode())
                self.write_msg = False
                self.msg = "Slider: "
                self.write_move = False
                self.write_shift = False
                self.write_timeout = False
                self.write_stop = False
            self.read()
            self.pub_busy.publish(self.busy)
            self.pub_pos.publish(self.pos)
            self.pub_slider_state.publish(self.state)
            sleep(1)


if __name__ == '__main__':
    rospy.init_node('slider_interface_node')
    ns = ""
    try:
        ns = rospy.get_param('~ns')
    except:
        pass
    slider = Slider_real_interface(ns)
    rospy.spin()
    '''
    try:
        while not rospy.is_shutdown():
            slider.read()
            #slider.pub_busy.publish(slider.busy)
            #slider.pub_pos.publish(slider.pos)
            sleep(1)
    except KeyboardInterrupt:
        pass
    '''