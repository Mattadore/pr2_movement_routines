#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <vector>
#include <cstdlib>
#include <string>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tester");
    /*moveit::planning_interface::PlanningSceneInterface planner;
    std::vector<std::string> names = planner.getKnownObjectNames();
    ROS_INFO("%s","listing members");
    for (int i=0;i<names.size();++i) {
        ROS_INFO("%s",names[i].c_str());
    }*/
    //system("rm -rf ./rosbag_stuff/*"); //empty rosbag stuff
    std::string filepath = ros::package::getPath("pr2_movement_routines");
    system(("rm -rf " + filepath + "/rosbag_stuff/*").c_str()); //empty rosbag stuff

}
