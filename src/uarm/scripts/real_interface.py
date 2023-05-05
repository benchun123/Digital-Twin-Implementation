#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
import threading
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from uarm.wrapper import SwiftAPI
from uarm.tools.list_ports import get_ports
from flask import Flask, request

"""
1. listen and auto connect all filter uArm
2. auto sync move (only enforce sync on connect a new uArm)
"""

swift = None
speed = 1000000
timeout = None

#maximum coordinates
x_min = 150
y_min = 150
w_min = 0
x_max = 280
y_max = 280
z_max = 130
w_max = 180


def find_swift():
    global swift

    found_swift = False

    for i in range(10):
        try:
            swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})

            swift.waiting_ready(timeout=3)

            found_swift = True
            break
        except:
            pass

    if not found_swift:
        print("could not connect to swift")
        return

    device_info = swift.get_device_info()
    print(device_info)
    firmware_version = device_info['firmware_version']
    if firmware_version and not firmware_version.startswith(('0.', '1.', '2.', '3.')):
        swift.set_speed_factor(0.0005)
    swift.set_mode(0)
    swift.reset(wait=True, speed=speed)
    

app = Flask(__name__)

@app.route('/move')
def index():
    #initialize all variables
    
    pump = False
    x1 = 0
    y1 = 0
    z1 = 0
    a1 = 0
    x2 = 0
    y2 = 0
    z2 = 0
    a2 = 0

    #read input
    task = str(request.json["task"])
    '''
    x1 = int(request.json["x1"])  #0 bis 280
    y1 = int(request.json["y1"])  #0 bis 280
    z1 = int(request.json["z1"])  #0 bis 150
    x2 = int(request.json["x2"])
    y2 = int(request.json["y2"])
    z2 = int(request.json["z2"])   
    
    #radius
 
    r1 = (x1**2+y1**2)**0.5
    r2 = (x2**2+y2**2)**0.5

    print("radius " + str(r1))
    '''

    #contol pump
    if task == "set_pump":
        pump = bool(request.json["pump"])
        if pump == True:
            swift.set_pump(on=True, wait=True)
        else:
            swift.set_pump(on=False, wait=True)
        return {'success' : True}

    elif task == "stop":
        print(task)
        try:
            swift.set_pump(on=False, wait=False)
            swift.set_buzzer(duration = 0.1, wait=True)
            '''
            swift.disconnect()
            time.sleep(3)      
            find_swift()
            '''
            pos = swift.get_position(wait = True)
            x = int(pos[0])
            y = int(pos[1])
            z = int(pos[2])
            swift.set_position(x=x, y=y, z=z, speed=speed, wait=False, timeout=timeout)
            swift.flush_cmd(wait_stop=True)
            swift.set_buzzer(duration = 0.1, wait=True)
        except:
            pass
        return {'success' : True}

    #moves to positon if valid
    elif task == "move_to":
        x1 = int(request.json["x1"])  #0 bis 280
        y1 = int(request.json["y1"])  #0 bis 280
        z1 = int(request.json["z1"])  #0 bis 150
        w1 = int(request.json["w1"])  #0 bis 180
        r1 = (x1**2+y1**2)**0.5
        print(task)
        print(x1)
        print(y1)
        print(z1)     
        print(w1)    
        if (r1 >= 150 and r1 <= 280 and x1 >= 30 and z1 <= z_max):

            pos = swift.get_position(wait = True)
            x = int(pos[0])
            y = int(pos[1])
            print("(x/y): " + "("+str(x)+"/"+str(y)+")") 
            #needed if travel distance is to big 
            extra_point = False
            if (((y < -180 and x < 100) and (y1 > 180 and x1 < 100)) or ((y > 180 and x < 100) and (y1 < -180 and x < 100))):
                extra_point = True
            if extra_point:
                swift.set_position(x=r1, y=0, z=z_max+20, speed=speed, wait=False, timeout=timeout)
            swift.set_wrist(angle=w1)
            swift.set_position(x=x1, y=y1, z=z1, speed=speed, wait=False, timeout=timeout)
            swift.flush_cmd(wait_stop=True)
            pos = swift.get_position(wait = True)
            xe = int(pos[0])
            ye = int(pos[1])
            ze = int(pos[2])
            print("uarm position:")
            print(pos)
            if(xe == x1 and ye == y1 and ze == z1):
                return {'success' : True}

            else:
                return {'success' : False} 
        else:
            for x in range(0, 3):
                swift.set_buzzer(duration = 0.1, wait=True)
                swift.flush_cmd(wait_stop=True)
                time.sleep(.1)
            print("uarm position:")
            print(swift.get_position(wait = True))
            return {'success' : False}

    #moves arm automaticly to position
    elif task == "auto_move":
        x1 = int(request.json["x1"])  #0 bis 280
        y1 = int(request.json["y1"])  #0 bis 280
        z1 = int(request.json["z1"])  #0 bis 150
        w1 = int(request.json["w1"])  #0 bis 180
        r1 = (x1**2+y1**2)**0.5
        print(task)
        print(x1)
        print(y1)
        print(z1)
        print(w1)    
        if (r1 >= 150 and r1 <= 280 and z1 <= z_max and x1 >= 30):    
           
            pos = swift.get_position(wait = True)
            x = int(pos[0])
            y = int(pos[1])
            print("(x/y): " + "("+str(x)+"/"+str(y)+")") 
            #needed if travel distance is to big 
            extra_point = False
            if (((y < -180 and x < 100) and (y1 > 180 and x1 < 100)) or ((y > 180 and x < 100) and (y1 < -180 and x < 100))):
                extra_point = True           
            swift.set_position(x=x, y=y, z=z_max, speed=speed, wait=True, timeout=timeout)
            
            swift.flush_cmd(wait_stop=True)
            if extra_point:
                swift.set_position(x=r1, y=0, z=z_max+20, speed=speed, wait=False, timeout=timeout)
                #swift.flush_cmd(wait_stop=False)
                #time.sleep(.1)
            #deliver
            swift.set_position(x=x1, y=y1, z=z_max, speed=speed, wait=True, timeout=timeout)
            swift.set_wrist(angle=w1)            
            swift.flush_cmd(wait_stop=True)
            swift.set_position(x=x1, y=y1, z=z1, speed=speed, wait=False, timeout=timeout)
            swift.flush_cmd(wait_stop=True)
            swift.flush_cmd(wait_stop=True)

            pos = swift.get_position(wait = True)
            xe = int(pos[0])
            ye = int(pos[1])
            ze = int(pos[2])
            print("uarm position:")
            print(pos)
            if(xe == x1 and ye == y1 and ze == z1):
                return {'success' : True}

            else:
                return {'success' : False}
        else:
            for x in range(0, 3):
                swift.set_buzzer(duration = 0.1, wait=True)
                swift.flush_cmd(wait_stop=True)
                time.sleep(.1)
            print("uarm position:")
            print(swift.get_position(wait = True))
            return {'success' : False}
    #recalibrates swift
    elif task == "reset":
        print(task)
        swift.set_pump(on=False, wait=True)
        '''
        swift.disconnect()
        time.sleep(3)      
        find_swift()
        '''
        swift.reset(wait=True, speed=speed)
        swift.set_buzzer(duration = 0.1, wait=True)
        print("uarm position:")
        print(swift.get_position(wait = True))
        return {'success' : True}

    elif task == "get_state":
        print(task)
        pump_state = swift.get_pump_status(wait = True)
        if pump_state == 0:
            pump = False
        else:
            pump = True
        pos = swift.get_position(wait = True)
        x = int(pos[0])
        y = int(pos[1])
        z = int(pos[2])
        return{'pump': pump, 'x': x, 'y': y, 'z': z}
    else:
        pass

if __name__ == '__main__':
    find_swift()

    app.run(debug=False)


'''
#transports cargo between two positions
elif task == "auto_transfer":
    x1 = int(request.json["x1"])  #0 bis 280
    y1 = int(request.json["y1"])  #0 bis 280
    z1 = int(request.json["z1"])  #0 bis 150
    w1 = int(request.json["w1"])  #0 bis 180
    x2 = int(request.json["x2"])
    y2 = int(request.json["y2"])
    z2 = int(request.json["z2"])
    w2 = int(request.json["w2"])

    r1 = (x1**2+y1**2)**0.5
    r2 = (x2**2+y2**2)**0.5

    if (r1 >= 150 and r1 <= 280 and r2 >= 150 and r2 <= 280 and z1 < (z_max-40) and z2 < (z_max-30) and x1 >= 30 and x2 >= 30):    
        
        #pick up
        swift.set_position(x=x1, y=y1, z=z_max, speed=speed, wait=False, timeout=timeout)
        swift.set_wrist(angle=w1)
        swift.set_position(x=x1, y=y1, z=z1, speed=speed, wait=False, timeout=timeout)
        swift.flush_cmd(wait_stop=True)
        swift.set_pump(on=True, wait=True)
        #time.sleep(3)
        time.sleep(.2)
        swift.set_position(x=x1, y=y1, z=z_max, speed=speed, wait=False, timeout=timeout)
        #deliver
        swift.set_position(x=x2, y=y2, z=z_max, speed=speed, wait=False, timeout=timeout)
        swift.set_wrist(angle=w2)
        swift.set_position(x=x2, y=y2, z=z2, speed=speed, wait=False, timeout=timeout)
        swift.flush_cmd(wait_stop=True)
        #time.sleep(.5)
        swift.set_pump(on=False, wait=True)
        swift.set_position(x=x2, y=y2, z=z_max, speed=speed, wait=True, timeout=timeout)
        swift.flush_cmd(wait_stop=True)
        return {'success' : True}

    else:
        for x in range(0, 3):
            swift.set_buzzer(duration = 0.1, wait=True)
            swift.flush_cmd(wait_stop=True)
            time.sleep(.1)
        return {'success' : False}
'''