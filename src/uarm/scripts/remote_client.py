#!/usr/bin/env python

import rospy
import uarm_msgs.msg
from uarm_msgs.srv import *
from std_srvs.srv import *
from action_client import ActionClient
from std_msgs.msg import Bool
import actionlib
import time

##################################################################
#               methods that call the services                   #
##################################################################

#call uarm_move service

#################################
#                               #
#       valid positions         #
#                               #
#      x element [30, 280]      #
#                               #
#      y element [0, 280]       #
#                               #
#      z element [-120, 130]    #
#                               #
#      w element [0, 180]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#  ATTENTION: Platform at z=0   #
#                               #
#################################

class uArm_Remote_Client():

    def __init__(self,ns):
        self.busy = False
        self.name = ns
        self.client_move = ActionClient(str(self.name+'uarm_move'))
        self.client_reset = ActionClient(str(self.name+'uarm_reset'))
        self.client_auto = ActionClient(str(self.name+'uarm_auto_move'))
        self.status_listener = rospy.Subscriber(str(self.name+"busy"), Bool, self.callback)
        self.ranges = '''
#################################
#                               #
#       valid positions         #
#                               #
#      x element [30, 280]      #
#                               #
#      y element [-280, 280]    #
#                               #
#      z element [-120, 130]    #
#                               #
#      w element [0, 180]       #
#                               #
#  RADIUS in x-y Ebene > 150    #
#                               #
#  ATTENTION: Platform at z=0   #
#                               #
#################################'''
        print(self.ranges)

    def callback(self, data):
        self.busy = data.data

    def move_client(self, x, y, z, w):
        if not self.busy:  
            return self.client_move.move(x=x, y=y, z=z, w=w)  
        else:
            return False


    def move(self):
        print("select coordinates of target position")
        try:
            x = int(input("x: "))
            y = int(input("y: "))
            z = int(input("z: "))
            w = int(input("w: "))
        except:
            return False

        return self.move_client(x, y, z, w)
    #call uarm_pump service
    def pump(self):
        print("select pump state True or False")
        pump_state = False
        if input("True 1 / False 0:  ") == 1:
            pump_state = True
            
        rospy.wait_for_service(str(self.name+'uarm_set_pump'))
        print("connected")
        try:
            service_object = rospy.ServiceProxy(str(self.name+'uarm_set_pump'), uarm_pump)
            received_response = service_object(pump_state)
            print("service called")
            return received_response.success
        except:
            print("service call failed")
            return(False)


    def reset_client(self):
        if not self.busy:  
            return self.client_reset.move(x=0, y=0, z=0, w=0)  
        else:
            return False

    #call the uarm_get_state service
    def state(self):
        print("reading uarm state")
        rospy.wait_for_service(str(self.name+'uarm_get_state'))
        try:
            service_object = rospy.ServiceProxy(str(self.name+'uarm_get_state'), uarm_get_state)
            received_response = service_object()
            return received_response
        except:
            print("service call failed")
            return(False)

    def auto_move_client(self, x, y, z, w):
        if not self.busy:  
            return self.client_auto.move(x=x, y=y, z=z, w=w)  
        else:
            return False


    #call uarm_auto_move service
    #here with explenations how to call a service
    def auto_move(self):
        print("select coordinates of target position")
        try:
            x1 = int(input("x: "))
            y1 = int(input("y: "))
            z1 = int(input("z: "))
            w1 = int(input("w: "))
        except:
            return False
        
        #search channel

        return self.auto_move_client(x1, y1, z1, w1)

    def stop(self, data=None):
        rospy.wait_for_service(str(self.name+'stop'))
        print("connected")
        try:
            service_object = rospy.ServiceProxy(str(self.name+'stop'), Trigger)
            received_response = service_object()
            print("service called")
            return received_response.success
        except:
            print("service call failed")
            return(False)




if __name__ == "__main__":
    rospy.init_node('uarm_client', anonymous=True)

    ns = "uarm1/"

    try:
        ns = rospy.get_param('~ns_client')
    except:
        pass
    server = uArm_Remote_Client(ns)
    #time.sleep(5)

    ##########################################################################
    #                      runtime loop                                      #
    ##########################################################################

    try:
        waiting_counter = 0
        while not rospy.is_shutdown():
            if not server.busy:
                waiting_counter = 0
                print("Options:")
                print("1: move arm to position")
                print("2: set pump")
                print("3: reset uarm")
                print("4: get state")
                print("5: auto move (straight up, then to target position)")
                print("6: quit")
                print("7: instrucions/ valid positions")
                print("8: stop")
                
                try:
                    option = int(input(">> "))
                except:
                    option = 0

                if option == 1:
                    print(server.move())

                elif option == 2:
                    print(server.pump())

                elif option == 3:
                    print(server.reset_client())

                elif option == 4:
                    print(server.state())

                elif option == 5:
                    print(server.auto_move())

                elif option == 6:
                    break

                elif option == 7:
                    print(server.ranges)

                elif option == 8:
                    print(server.stop())
                else:
                    pass
            else:
                time.sleep(2)
                waiting_counter += 1
                print("waiting for uarm to be ready...")
            
    except KeyboardInterrupt:
        pass


