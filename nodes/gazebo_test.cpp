
// Software License Agreement (BSD License)

// Copyright (c) 2016, Matthew Buckley
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// author: Matthew Buckley

#include <ros/ros.h>
#include <simple_robot_control/robot_control.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <sys/types.h>
#include <std_msgs/String.h>
#include <boost/shared_ptr.hpp>
#include <cstdlib>
#include <particle_filter/action.h>
#include <particle_filter/handle_finder.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <string>
#include <moveit_msgs/Grasp.h>
#include <unistd.h>
#include <ros/package.h>
#include <pthread.h>


simple_robot_control::Robot * robot;
tf::TransformListener *tf_listener;

void moveTo(tf::Point & displacement,double speed = 0.25) { //go to location
    displacement.setZ(0); //flatten
    double distance = displacement.length();
    displacement.normalize();
    displacement*=speed;
    geometry_msgs::Twist velocity;
    geometry_msgs::Vector3 velocityvec,angular;
    tf::vector3TFToMsg(displacement,velocityvec);
    velocity.linear = velocityvec;
    angular.x=0;
    angular.y=0;
    angular.z=0;
    velocity.angular = angular;
    robot->base.drive(distance, velocity);
}

void align_initial() { //line up with the object we plan to interact with
    //gets the location
    tf::Stamped<tf::Point> rawlocation,location;
    rawlocation = tf::Stamped<tf::Point>(tf::Point(0,0,0.244),ros::Time(0),"ar_marker_6");
    tf_listener->transformPoint("base_footprint",rawlocation,location);
    tf::StampedTransform footToGripper;
    tf_listener->lookupTransform("base_footprint","r_wrist_roll_link" ,  ros::Time(0), footToGripper);
    location -= ((tf::Transform)footToGripper).getOrigin();
    moveTo(location);
}

void move_dir(Action & act,tf::Vector3 & dir, const std::string & markername) { //move hand in direction
    robot->right_gripper.close();
    bool both = true;
    act.execute(dir,markername,both);
}

tf::Vector3 align_pos_l(0.5,0.2,1);
tf::Vector3 align_pos_r(0.5,-0.2,1);

void * restLArm(void * args) {
    robot->left_arm.moveGripperToPosition(align_pos_l,"base_link",simple_robot_control::Arm::FROM_BELOW);
}

void * restRArm(void * args) {
    robot->right_arm.moveGripperToPosition(align_pos_r,"base_link",simple_robot_control::Arm::FROM_BELOW);
}

void * moveBaseToObj(void * args) {
    robot->right_gripper.open();
    align_initial();
    align_initial();
    align_initial();
}

void * openGripper(void * args) {
    robot->right_gripper.open();
    double wide;
    robot->right_gripper.getOpeningWidth(wide);
    while (wide < 0.07) {
        //bool worked = robot->left_gripper.open();
        ros::Duration(0.3).sleep(); //test value?
        robot->right_gripper.getOpeningWidth(wide);
    }
}

void * repositionRArm(void * args) {
    tf::Stamped<tf::Point> grab,rawgrab = tf::Stamped<tf::Point>(tf::Point(0.2,0,0.039),ros::Time(0),"ar_marker_6");
    tf_listener->transformPoint("base_footprint",rawgrab,grab);
    robot->right_arm.moveGripperToPosition(grab,"base_link");
}

//void * grabIt(void * args) {
//    tf::Stamped<tf::Point> grab,rawgrab = tf::Stamped<tf::Point>(tf::Point(0,-0.011,-0.1),ros::Time(0),"ar_marker_6");
//    tf_listener->transformPoint("base_footprint",rawgrab,grab);
//    robot->right_arm.moveGripperToPosition/*WithCollisionChecking*/(grab,"base_link");
//    robot->right_gripper.close();
//}

int main(int argc, char** argv) {

    ros::init(argc, argv, "gazebo_routines");
    //ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    ros::NodeHandle n, pn("~");
    ros::NodeHandle handle;
    ros::Publisher flowpub = handle.advertise<std_msgs::String>("flowManager", 10);
    std_msgs::String sendstr;
    sendstr.data = "Start moves";
    flowpub.publish(sendstr);
	tf_listener = new tf::TransformListener(n);
    ros::Duration(0.5).sleep();
    robot = new simple_robot_control::Robot();
    void * status;
    
    //use threads to speed up process a bit by allowing robot to get joints in position together
    pthread_t thread[3];
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    
    pthread_create(&thread[0], &attr, restLArm, NULL); 
    pthread_create(&thread[1], &attr, restRArm, NULL); 
    pthread_create(&thread[2], &attr, moveBaseToObj, NULL); 

    pthread_join(thread[0], &status);
    pthread_join(thread[1], &status);
    pthread_join(thread[2], &status);
    Action act;

    pthread_create(&thread[0], &attr, openGripper, NULL); 
    pthread_create(&thread[1], &attr, repositionRArm, NULL);
    
    pthread_join(thread[0], &status);
    pthread_join(thread[1], &status);
    
    tf::Stamped<tf::Point> grab,rawgrab = tf::Stamped<tf::Point>(tf::Point(0.2,0,-0.036),ros::Time(0),"ar_marker_6");
    tf_listener->transformPoint("base_footprint",rawgrab,grab);
    
    robot->right_arm.moveGripperToPosition(grab,"base_link");

    robot->right_gripper.close();
    //TODO: use their handle grasping methods
    ros::Duration(4).sleep(); //test value?
    sendstr.data = "Start recording";
    flowpub.publish(sendstr);
    ROS_INFO("%s","object grasped");
    std::string filepath = ros::package::getPath("pr2_movement_routines");
    system(("rm -rf " + filepath + "/rosbag_stuff/*").c_str()); //empty rosbag stuff
    const std::string markername("ar_marker_6");
    tf::Vector3 dir;
    { //do appropriate movements here
        dir = tf::Vector3(0,4.5,0);
        move_dir(act,dir,markername);
        ROS_INFO("%s","Completed y-axis movement");
        /*dir = tf::Vector3(1.5,0,0); //this is semi annoying
        move_dir(act,dir,markername);
        ROS_INFO("%s","Completed x-axis movement");
        dir = tf::Vector3(0,0,1.5);
        move_dir(act,dir,markername);
        ROS_INFO("%s","Completed z-axis movement");*/ //alternative movement routines
    }
    ROS_INFO("%s","Completed Movements");
    sendstr.data = "Stop recording";
    flowpub.publish(sendstr);
    pthread_attr_destroy(&attr);
    delete robot;
    delete tf_listener;
    pthread_exit(NULL);
}
