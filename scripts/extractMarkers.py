#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Karol Hausman
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
# author: Karol Hausman

import rospy 
import numpy as np
import matplotlib.pyplot as plt
import pickle
from pr2_lfd_utils import generalUtils
from pr2_lfd_utils import trajUtils
from pr2_lfd_utils import arWorldModel
from pr2_lfd_utils import drawUtils
from sensor_msgs.msg import *
from std_msgs.msg import String
import sys
import os
import os.path
import time
import matplotlib.pyplot as plt
from numpy import linalg as LA
import rospkg

from articulation_model_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

global ended
ended = False

           
def run():
    global r,traj_utils,draw_utils,wm,marker_ids,skill_id
    #Setup utilities and world model
    #gen_utils = generalUtils.GeneralUtils()

    #Construct file names
#        basename = 'data/bagfiles/2014_8_7_22_5_46/'
#        basename = 'data/bagfiles/2014_9_7_23_38_42/'
#        basename = 'data/bagfiles/2014_8_21_10_46_5/'
#        basename = 'data/bagfiles/2014_8_10_21_37_48/'
#        basename = 'data/bagfiles/2014_9_27_17_46_0/'
    rospack = rospkg.RosPack()
    basename = rospack.get_path('pr2_movement_routines')+'/rosbag_stuff/' #sorry :/
    markerfile = basename + 'Marker' + str(skill_id) + '.txt'

    marker_goal_data = []
    marker_difference_data = []
    print len(marker_ids)
    for i in range (len(marker_ids)):
        print "extracting marker IDs: ", marker_ids[i]
        marker_goal_data.append(traj_utils.createMarkerTrajFromFile(markerfile, int(marker_ids[i]), -1))
        if ((i%2) == 1):
            marker_difference_data.append(traj_utils.createDifferenceTrajFromFile(markerfile, int(marker_ids[i-1]), int(marker_ids[i])))
            [diff, drawable] = traj_utils.createDifferenceTrajFromFile(markerfile, int(marker_ids[i-1]), int(marker_ids[i]))


    dist_data = []
    for i in range (len(diff)):
        dist_data.append( LA.norm( diff[i][0:3] ) )

    print "dist_data: mean: ", np.mean(dist_data),  "median: ", np.median(dist_data), "std: ", np.std(dist_data)
    print rospy.is_shutdown()
    
    #1D
    #draw_utils.plotTraj(drawable, 0.1)      

    #draw_utils.plotTwoTraj3D(marker_goal_data[0], marker_goal_data[1])
    
    #3D single
    #draw_utils.plotTraj3D(marker_goal_data[0])
    #draw_utils.plotTraj3D(marker_goal_data[1])
    #3D        
    #draw_utils.plotTraj3D(diff)
    
    #plt.show()
#        print "marker goal data: ", marker_goal_data

    #publish all the markers
    pub = rospy.Publisher('marker_topic', TrackMsg)
    msg = TrackMsg()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "/world"
    #for i in marker_goal_data[0]:
    incremental = False
    if (not incremental):
        
        f = open('pickle.txt','w')                    
        for i in diff:    
            pose = Pose(Point(i[0], i[1], i[2]), Quaternion(i[3], i[4], i[5], i[6]))
            msg.pose.append( pose )
            
            for item in i:
                f.write("%f, " % item) # python will convert \n to os.linesep
            f.write('\n')            

        #f.close()
        while not rospy.is_shutdown():
            pub.publish(msg)
            time.sleep(0.1)
            #r.sleep()
    else:
        cnt = 0
        while not rospy.is_shutdown():
            msg.pose = []
            i = diff[cnt]
            pose = Pose(Point(i[0], i[1], i[2]), Quaternion(i[3], i[4], i[5], i[6]))
            msg.pose.append( pose )
            cnt = cnt + 1
            pub.publish(msg)
            time.sleep(0.1)
            #r.sleep()
            


def messageCallback(msg):
    if msg.data == "Stopped converting":
        print "Started extracting"
        print_str = "Started extracting"
        global pub
        pub.publish(print_str)
        run()
        global ended
        ended = True
    
if __name__ == '__main__':
    try:
        global pub,r,traj_utils,draw_utils,wm,marker_ids,skill_id
        rospy.init_node('ExtractMarkersNode')
        r = rospy.Rate(10) # 10hz
        traj_utils = trajUtils.TrajUtils()
        draw_utils = drawUtils.DrawUtils()
        wm = arWorldModel.ARWorldModel()
        marker_ids = []
        if (len(sys.argv) >= 3):
            skill_id = int(sys.argv[1])
            for id in sys.argv[2:4]:
              marker_ids.append(id)
        else:
            print "\nAborting! Wrong number of command line args"
            print "Usage: python extractMarkers <skill_id> <marker_id> <marker_id> <marker_id> ..."
            sys.exit(0)
            
        pub = rospy.Publisher("flowManager",std_msgs.msg.String)
        rospy.Subscriber("flowManager", std_msgs.msg.String, messageCallback)
        r = rospy.Rate(10)
        print rospy.is_shutdown()
        while not (rospy.is_shutdown() or ended):
            r.sleep()
            
        if (rospy.is_shutdown()):
            sys.exit(0)

   
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
