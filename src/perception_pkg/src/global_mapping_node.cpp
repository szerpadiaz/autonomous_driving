#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <octomap_msgs/conversions.h>

class global_mapping_node{
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub;
    ros::Publisher octomap_pub;
    ros::Publisher octomap_2D_pub;
    octomap::OcTree octree;

public:
    global_mapping_node() : octree(1) {
        point_cloud_sub = nh.subscribe("points_cloud", 1000, &global_mapping_node::point_cloud_cb, this);
        octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 10);
        octomap_2D_pub = nh.advertise<octomap_msgs::Octomap>("octomap2D", 10);
    }

    void point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        
        // Convert msg into a octomap's point-cloud 
        octomap::Pointcloud octo_cloud;

        const unsigned char* data = msg->data.data();
        int total_points = msg->width * msg->height;
        int step = msg->point_step;
        for (int i = 0; i < total_points; ++i) {
            float x = *(float*)(data + i * step + msg->fields[0].offset);
            float y = *(float*)(data + i * step + msg->fields[1].offset);
            float z = *(float*)(data + i * step + msg->fields[2].offset);

            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                octo_cloud.push_back(x, y, z);
                //ROS_INFO_NAMED("global mapping", "x = %f ; y = %f ; z = %f", x, y, z);
            }
            else{
                //ROS_INFO_NAMED("global mapping", "INFINITE!!!");
            }
        }
        
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::fromROSMsg(*msg, *cloud);
        //for(const auto& point : cloud->points){
        //    octo_cloud.push_back(point.x, point.y, point.z);
        //}

        // Build octomap's tree
        octree.insertPointCloud(octo_cloud, octomap::point3d(0,0,0));

        // Publish octomap
        octomap_msgs::Octomap map_msg;
        octomap_msgs::fullMapToMsg(octree, map_msg);
        map_msg.header.frame_id = "OurCar/Sensors/DepthCamera";
        octomap_pub.publish(map_msg);

        // Convet 3D map into a 2D map
        octomap::OcTree::tree_iterator it;
        octomap::OcTree map_2D(octree.getResolution());
        for (it = octree.begin_tree(); it != octree.end_tree(); ++it) {
            if (octree.isNodeOccupied(*it)) {
                map_2D.updateNode(it.getCoordinate(), true);
            }
        }

        // Publish octomap
        octomap_msgs::Octomap map2D_msg;
        octomap_msgs::binaryMapToMsg(octree, map2D_msg);
        map2D_msg.header.frame_id = "OurCar/Sensors/DepthCamera";
        octomap_2D_pub.publish(map2D_msg);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "global_mapping_node");
    ROS_INFO_NAMED("global mapping", "started!");
    global_mapping_node n;
    ros::spin();
}