#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

class Localization{
    private:
        ros::NodeHandle nh;
        float current_x , current_y, current_z;
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;
        float roll_ , pitch_ , yaw_ ;


    public:
        Localization(){

        }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "localization_node");
    ROS_INFO_NAMED("localization_node", "started!");
    Localization n;
    ros::spin();
}