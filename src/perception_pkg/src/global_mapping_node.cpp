#include <ros/ros.h>

class global_mapping_node{
    ros::NodeHandle nh;
public:
    global_mapping_node() {

    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "global_mapping_node");
    ROS_INFO_NAMED("global mapping", "started!");
    global_mapping_node n;
    ros::spin();
}