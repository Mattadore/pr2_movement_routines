#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Scott Niekum
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Scott Niekum

#import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy 
import numpy as np
import pr2_mechanism_msgs.srv
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import * 
from sensor_msgs.msg import *
from std_msgs.msg import String
import rosbag
import subprocess
import os
import signal
import time
import rospkg

class RecordInteraction():
    
    def __init__(self, base_path, joint_thresh=0.001):
        
        #rospy.wait_for_service('pr2_controller_manager/switch_controller')
        #self.switch_control = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pr2_mechanism_msgs.srv.SwitchController, persistent=True)
        #print 'Recorder registered to manager'

        #self.standard_controllers = ['r_arm_controller', 'l_arm_controller']
        #self.mannequin_controllers = ['r_arm_controller_loose', 'l_arm_controller_loose']
        self.whicharm_mann = 0 #right arm?
        
        #self.switch_req = pr2_mechanism_msgs.srv.SwitchControllerRequest()
        #self.switch_req.strictness = pr2_mechanism_msgs.srv.SwitchControllerRequest.BEST_EFFORT
        
        #Only change fixed point if some joint moves at least joint_thresh radians
        #self.joint_bounds = [joint_thresh]*10
        #self.interaction = False
        
        #self.r_pub = rospy.Publisher("/r_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
        #rospy.Subscriber("/r_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.rightJointStateCallback)
        #self.l_pub = rospy.Publisher("/l_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
        #rospy.Subscriber("/l_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.leftJointStateCallback)
        self.stopnode = False
        self.bag_process = None
        self.recording = False
        self.base_path = base_path
        self.file_path = base_path
        self.seg_num = 1
        self.pub = rospy.Publisher("flowManager",std_msgs.msg.String)
        rospy.Subscriber("flowManager", std_msgs.msg.String, self.flowCallback)
        
    def __del__(self):
        #Make sure to stop recording bag before shutdown
        self.stopRecord()
    #Start interaction by stopping standard controllers and going into mannequin mode
    #Starts with left arm mannequin, right arm rigid
            
    #def switchToRightArm(self):
    #    if self.interaction:
    #        if self.whicharm_mann == 1:
    #            print "right"
    #            self.switch_req.stop_controllers = [self.mannequin_controllers[1], self.standard_controllers[0]]
    #            self.switch_req.start_controllers = [self.standard_controllers[1], self.mannequin_controllers[0]]
    #            resp = self.switch_control(self.switch_req)
    #            self.switchRecord()
    #            self.whicharm_mann = 0
    
    
    #def switchToLeftArm(self):
    #    if self.interaction:
    #        if self.whicharm_mann == 0:
    #            print "left"
    #            self.switch_req.stop_controllers = [self.mannequin_controllers[0], self.standard_controllers[1]]
    #            self.switch_req.start_controllers = [self.standard_controllers[0], self.mannequin_controllers[1]]
    #            resp = self.switch_control(self.switch_req)
    #            self.switchRecord()
    #            self.whicharm_mann = 1
    
    def flowCallback(self,msg):
        print "Message received"
        print msg.data
        if msg.data == "Start recording":
            if not self.recording:
                print "Python started recording"
                self.startRecord()
                print_str = "Started recording"
                self.pub.publish(print_str)
        elif msg.data == "Stop recording":
            if self.recording:
                print "Python stopped recording"
                self.stopRecord()
                self.stopnode = True
                print_str = "Stopped recording"
                self.pub.publish(print_str)
    
    def startRecord(self):
        if not self.recording:
            self.seg_num = 1
            #Make a new folder for the demo named by date/time 
            
            print "Beginning to record."
            self.recording = True
            
            topics = "/ar_world_model /l_arm_controller/state /r_arm_controller/state /l_gripper_controller/state /r_gripper_controller/state"
            filename = self.file_path+"/part" + str(self.seg_num) + ".bag " 
            command = "rosbag record -O " + filename + topics
            self.bag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)       
         
    def stopRecord(self):
        if self.recording:
            self.recording = False
            
            #Make sure we kill the child process that rosbag spawns in addition to the parent subprocess
            pid = self.bag_process.pid
            os.killpg(pid, signal.SIGINT)
            print "Stopping recording.\n"

if __name__ == '__main__':
    rospy.init_node('recordInteractionNode')
    
    rc = RecordInteraction(rospack.get_path('pr2_movement_routines')+'/rosbag_stuff')
    
    r = rospy.Rate(10)
    while not (rospy.is_shutdown() or rc.stopnode):
        r.sleep()
        