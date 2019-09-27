#!/usr/bin/env python

# for State Machine
from transitions import Machine
# from transitions.extensions import GraphMachine
import random
import numpy as np
import time

# for ROS
import rospy
from std_msgs.msg import String, Bool, Int32
from mission_planner_mbzirc.msg import block_pose, ur5_state, omobot_state

DEBUG = True

class Mission_Planner(object):
    states = ['idle', 'driving', 'loading', 'unloading']

    def __init__(self):
        # ======================================================== #
        # ==================== for Class data ==================== #
        # ======================================================== #
        self.mission_start = 0

        self.block_x   = None
        self.block_y   = None
        self.block_z   = None
        self.block_yaw = None
        self.depth     = None
 
        self.omobot_state  = 0  # 0 : nothing / 1 : arrived block_1 / 2 : arrived block_2 / 3 : arrived site
        self.ur5_state     = 0  # 0 : nothing / 1 : loaded block_1  / 2 : load block_2    / 3 : unloaded
        # self.ur5_unload_state = 0  # 0 : nothing / 5 : unload_move_xy / 6 : unload_move_vertical_down / 7 : unload_move_vertical_up / 8 : unload_move_to_grab

        # ================================================================ #
        # ==================== for ROS Initialization ==================== #
        # ================================================================ #
        rospy.init_node('simple_planner')
        # For ROS Subscribing
        # self.sub_real_sense = rospy.Subscriber('real_sense/block_pose', block_pose, self.callback_real_sense)
        self.sub_mission_start = rospy.Subscriber('/mission_start', Bool, self.callback_mission_start)

        # self.sub_real_sense = rospy.Subscriber('block_pose', block_pose, self.callback_real_sense)
        # self.sub_ur5        = rospy.Subscriber('ur5/state', ur5_state, self.callback_ur5)
        # self.sub_omobot     = rospy.Subscriber('omobot/state', omobot_state, self.callback_omobot)
        self.sub_ur5        = rospy.Subscriber('/ur5/state', Int32, self.callback_ur5)
        self.sub_omobot     = rospy.Subscriber('/omobot/state', Int32, self.callback_omobot)

        # For ROS Publishing
        # self.pub_real_sense = rospy.Publisher('real_sense/flag', Int32, queue_size=1)         # data flag (0 : nothing / 1 : x,y,z,yaw / 2 : depth)
        self.pub_ur5        = rospy.Publisher('/ur5/flag', Int32, queue_size=1)    # flag for action.  (0 : nothing / 1 : load      / 2 : unload)
        self.pub_omobot     = rospy.Publisher('/omobot/flag', Int32, queue_size=1) # flag for driving. (0 : nothing / 1 : to block_1 / 1 : to block_1 / 2 : to site)

        self.rate = rospy.Rate(10)

        # =================================================================== #
        # ==================== for Finitie State Machine ==================== #
        # =================================================================== #
        self.machine = Machine(model=self, states=Mission_Planner.states, initial='idle')

        # Add transitions
        self.machine.add_transition(trigger='drive', source='idle', dest='driving')
        self.machine.add_transition(trigger='arrived', source='driving', dest='idle')

        self.machine.add_transition(trigger='load_block', source='idle', dest='loading')
        self.machine.add_transition(trigger='loaded', source='loading', dest='idle')

        self.machine.add_transition(trigger='unload_block', source='idle', dest='unloading')
        self.machine.add_transition(trigger='unloaded', source='unloading', dest='idle')

    # ========================================================== #
    # ==================== for ROS Callback ==================== #
    # ========================================================== #
    def callback_mission_start(self, msg):
        if msg.data == 1:
            self.mission_start = True
        else:
            self.mission_start = False

    def callback_real_sense(self, msg):
        self.block_x = msg.x
        self.block_y = msg.y
        self.block_z = msg.z
        self.block_yaw = msg.yaw

        rospy.loginfo("Subscrived from real sense")

    def callback_ur5(self, msg):
        # self.ur5_state = msg.state
        self.ur5_state = msg.data  # 0 : nothing / 1 : loaded block 1 / 2 : loaded block 2 / 3 : unloaded

        # self.ur5_load_state = msg.load_state     # 0 : nothing / 1 : load_move_xy   / 2 : load_move_vertical_down   / 3 : load_move_vertical_up   / 4 : load_move_to_grab
        # self.ur5_unload_state = msg.unload_state # 0 : nothing / 5 : unload_move_xy / 6 : unload_move_vertical_down / 7 : unload_drop             / 8 : unload_move_to_idle

        rospy.loginfo("Subscrived from ur5")

    def callback_omobot(self, msg):
        # self.omobot_state = msg.state
        self.omobot_state = msg.data            # 0 : nothing / 1 : arrived blocks / 2 : arrived site

        rospy.loginfo("Subscrived from omobot")


def main():
    mission_planner = Mission_Planner()

    while not rospy.is_shutdown():

        # Mission Start Switch
        if mission_planner.mission_start == 0:
            continue
        else:
            # ========================== #
            # ====== State : idle ====== #
            # ========================== #
            if mission_planner.state == 'idle':

                # Drive
                if mission_planner.omobot_state == 0: # when robot is not blocks nor site
                    if mission_planner.ur5_state == 0:                                     # if ur5 is init
                        print("from idle to drive. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.drive()

                    else:                                                                  # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))        
                        mission_planner.pub_ur5.publish(0)

                # Load or Move to site at Block_1
                if mission_planner.omobot_state == 1: # at block_1
                    if mission_planner.ur5_state == 0:                                     # if ur5 is with nothing
                        print("from idle to load block_1. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.load_block()

                    elif mission_planner.ur5_state == 1:                                   # if ur5 finish loading at block_1
                        print("Finish loading block_1. Go block_2. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.drive()

                    else:                                                                  # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)

                # Load or Move to site at Block_2
                if mission_planner.omobot_state == 2: # at block_2
                    if mission_planner.ur5_state == 1:                                     # if ur5 is with block_1
                        print("from idle to load block_2. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.load_block()

                    elif mission_planner.ur5_state == 2:                                   # if ur5 finish loading at block_2
                        print("Finish loading block_2. Go site. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.drive()

                    else:                                                                  # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                
                # Unload or Move to blocks at Site
                if mission_planner.omobot_state == 3: # at site
                    if mission_planner.ur5_state == 2:                                     # if ur5 is with blocks
                        print("from idle to unload. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.unload_block()

                    elif mission_planner.ur5_state == 3:                                   # if ur5 finish unloading at site
                        print("Finish one sequence. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)

                    else:
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))                                         # re init robots
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)

            # =========================== #
            # ===== State : driving ===== #
            # =========================== #
            if mission_planner.state == 'driving':

                # Move to block_1 at init_pose
                if mission_planner.omobot_state == 0: # when robot is init or driving
                    if mission_planner.ur5_state == 0:        # if ur5 is init
                        print("Move to block_1. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(1)

                    else:
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_ur5.publish(0)    # re-initialize ur5 states
                        mission_planner.arrived()

                # Arrived or Move to block_2 at block_1
                if mission_planner.omobot_state == 1: # at block_1
                    if mission_planner.ur5_state == 0:        # if ur5 is init
                        print("Arrived block_1. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.arrived()

                    elif mission_planner.ur5_state == 1:      # if ur5 finish loading block
                        print("Move to block_2. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(2)

                    else:                                     # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.arrived()

                # Arrived or Move to site at block_2
                if mission_planner.omobot_state == 2: # at block_2
                    if mission_planner.ur5_state == 1:        # if ur5 arrived with block_1
                        print("Arrived block_2 with block_1. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.arrived()

                    elif mission_planner.ur5_state == 2:      # if ur5 finish loading block_2
                        print("Move to site. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(2)

                    else:                                     # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.arrived()

                # Arrived or Done one sequence
                if mission_planner.omobot_state == 3: # at site
                    if mission_planner.ur5_state == 2:        # if ur5 arrived site with blocks
                        print("Arrived site. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.arrived()

                    elif mission_planner.ur5_state == 3:      # if ur5 finish unloading (Done one sequence)
                        print("Done a sequence. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.arrived()

                    else:                                     # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.arrived()

            # =========================== #
            # ===== State : loading ===== #
            # =========================== #
            if mission_planner.state == 'loading':

                # Load block 1 or Move to block_2
                if mission_planner.omobot_state == 1: # at block_1
                    if mission_planner.ur5_state == 0:        # if ur5 is init
                        print("Load block_1. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_ur5.publish(1)

                    elif mission_planner.ur5_state == 1:      # if ur5 finish loading block
                        print("Finished loading. Re-init UR5. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.loaded()

                    else:                                     # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.loaded()

                # Load block_2 or Move to site
                if mission_planner.omobot_state == 2: # at block_2
                    if mission_planner.ur5_state == 1:        # if ur5 is init
                        print("Load block_2. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_ur5.publish(2)

                    elif mission_planner.ur5_state == 2:      # if ur5 finish loading block
                        print("Finished loading. Re-init UR5. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.loaded()

                    else:                                     # re init robots
                        print("Re initialization. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.loaded()

                # if mission_planner.ur5_state == 0: # Init state
                #     print("publish real sense flag 1 (x,y,z,yaw)")
                #     mission_planner.pub_ur5.publish(1)     # Move xy
                # elif mission_planner.ur5_load_state == 1: # Done Move xy
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(2)     # Move vertical down
                # elif mission_planner.ur5_load_state == 2: # Done Move vertical down
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(3)     # Move vertical up
                # elif mission_planner.ur5_load_state == 3: # Done Move vertical up
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(4)     # Move to grab
                # else: # mission_planner.ur5_load_state == 4: # Done Move to grab
                #     print("publish real sense flag 0 (init)")
                #     mission_planner.loaded()

            # ============================= #
            # ===== State : unloading ===== #
            # ============================= #
            if mission_planner.state == 'unloading':
                
                # Unload blocks or Done one sequence
                if mission_planner.omobot_state == 3: # at site
                    if mission_planner.ur5_state == 2:        # if ur5 is init
                        print("Unload blocks. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_ur5.publish(2)

                    elif mission_planner.ur5_state == 3:      # if ur5 finish unloading blocks
                        print("Finished unloading. (Planner: %s, UR5: %d, Omobot: %d)" %(mission_planner.state, mission_planner.ur5_state, mission_planner.omobot_state))
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.unloaded()

                    else:                                     # re init robots
                        print("Re initialization")
                        mission_planner.pub_omobot.publish(0)
                        mission_planner.pub_ur5.publish(0)
                        mission_planner.unloaded()


                # mission_planner.pub_ur5.publish()
                # if mission_planner.ur5_unload_state == 0: # Init state
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(5)     # Move xy
                # elif mission_planner.ur5_unload_state == 5: # Done Move xy
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(6)     # Move vertical down
                # elif mission_planner.ur5_unload_state == 6: # Done Move vertical down
                #     print("publish real sense flag 2 (depth)")
                #     mission_planner.pub_ur5.publish(7)     # Drop
                # elif mission_planner.ur5_unload_state == 7: # Done Drop
                #     print("publish real sense flag 0 (init)")
                #     mission_planner.pub_ur5.publish(8)     # Move to idle
                # else: # mission_planner.ur5_unload_state == 8: # Done Move to idle
                #     print("publish real sense flag 0 (init)")
                #     mission_planner.unloaded()

            mission_planner.rate.sleep()

if __name__ == '__main__':
    main()