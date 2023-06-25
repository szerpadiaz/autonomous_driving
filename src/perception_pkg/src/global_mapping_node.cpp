#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_msgs/conversions.h>
#include <tf/transform_listener.h>


class global_mapping_node{
    ros::NodeHandle nh;
    tf::StampedTransform  world_transform;
    ros::Subscriber point_cloud_sub;
    ros::Publisher octomap_pub;
    ros::Publisher occupancy_grid_pub;
    ros::Publisher transformed_point_cloud_pub;
    ros::Subscriber octomap_full_sub;

    octomap::OcTree octree;
    float rel_min_x = -5;
    float rel_max_x = 5;
    float rel_min_y = 0;
    float rel_max_y = 10;
    float rel_min_z = 1.0;
    float rel_max_z = 5.0;
    float resolution = 1.0;
    float occupancy_threshold = 0.5; 
    float occupancy_probability_hit = 0.6;
    float occupancy_probability_miss = 0.4;
    float occupancy_clamping_min = 0.1;
    float occupancy_clamping_max = 0.9;
    float abs_min_x = -60;
    float abs_min_y = -60;
    uint32_t grid_width = 600;
    uint32_t grid_height = 600;
    float grid_origin_x = -150;
    float grid_origin_y = -100;

public:
    global_mapping_node() : octree(1) {
        transformed_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
        //point_cloud_sub = nh.subscribe("points_cloud", 1, &global_mapping_node::transform_point_cloud_cb, this);
        octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
        occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1);

        octomap_full_sub = nh.subscribe("octomap_full", 1, &global_mapping_node::transform_3d_into_2d_map, this);
        octomap::OcTree octree(resolution);
        octree.setOccupancyThres(occupancy_threshold);
        octree.setProbHit(occupancy_probability_hit);
        octree.setProbMiss(occupancy_probability_miss);
        octree.setClampingThresMin(occupancy_clamping_min);
        octree.setClampingThresMax(occupancy_clamping_max);
    }

    bool update_world_transform(){
        bool success = false;
        tf::TransformListener tf_listener;
        try {
            tf_listener.waitForTransform("world", "camera", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("world", "camera", ros::Time(0), world_transform);
            success = true;
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to lookup world-frame transform: %s", ex.what());
        }
        return success;
    }

    void transform_3d_into_2d_map(const octomap_msgs::Octomap::ConstPtr& msg){
        // Convert the octomap message to an octomap object
        auto abstract_octree = octomap_msgs::msgToMap(*msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_octree);

        // Initialize variables for size and maximum values
        size_t numNodes = octree->size();
        double maxX = -std::numeric_limits<double>::max();
        double maxY = -std::numeric_limits<double>::max();
        double maxZ = -std::numeric_limits<double>::max();

        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double minZ = std::numeric_limits<double>::max();

        tf::TransformListener tf_listener;
        tf::StampedTransform  cam_transform;
        try {
            tf_listener.waitForTransform("camera", "world", ros::Time(0), ros::Duration(3.0));
            tf_listener.lookupTransform("camera", "world", ros::Time(0), cam_transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to lookup world-to-cam-frame transform: %s", ex.what());
        }

        // Iterate over the leaf nodes of the octomap
        for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            // Convert from world-frame to camera-frame
            tf::Vector3 map_point(it.getX(), it.getY(), it.getZ());
            //tf::Vector3 cam_point = cam_transform * map_point;
            tf::Vector3 cam_point =  map_point;
            
            minX = std::min(minX, cam_point.x());
            minY = std::min(minY, cam_point.y());
            minZ = std::min(minZ, cam_point.z());

            maxX = std::max(maxX, cam_point.x());
            maxY = std::max(maxY, cam_point.y());
            maxZ = std::max(maxZ, cam_point.z());
        }

        // Print the size and maximum values
        ROS_INFO("Octomap Size: %zu", numNodes);
        ROS_INFO("Min X: %f , Max X: %f", minX, maxX);
        ROS_INFO("Min Y: %f , Max Y: %f", minY, maxY);
        ROS_INFO("Min Z: %f , Max Z: %f", minZ, maxZ);

        minY = -100;
        minZ = -100;
        // Create an occupancy grid
        nav_msgs::OccupancyGrid occupancy_grid_msg;
        occupancy_grid_msg.header.frame_id = "world";
        occupancy_grid_msg.header.stamp = ros::Time::now();
        occupancy_grid_msg.info.resolution = 1;
        occupancy_grid_msg.info.width = 300; //(maxY - minY);
        occupancy_grid_msg.info.height = 300; //(maxZ - minZ) ;
        occupancy_grid_msg.info.origin.position.x = 0;
        occupancy_grid_msg.info.origin.position.y = minY;
        occupancy_grid_msg.info.origin.position.z = minZ;
        occupancy_grid_msg.info.origin.orientation.w = 1;
        occupancy_grid_msg.data = std::vector<int8_t>((occupancy_grid_msg.info.width * occupancy_grid_msg.info.height), -1);

        for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            if (octree->isNodeOccupied(*it)) {
                // Convert from world-frame to camera-frame
                tf::Vector3 map_point(it.getX(), it.getY(), it.getZ());
                //tf::Vector3 cam_point = cam_transform * map_point;
                tf::Vector3 cam_point =  map_point;

                // Convert the 3D point to grid indices
                int grid_y = (cam_point.y() - minY) / occupancy_grid_msg.info.resolution;
                int grid_z = (cam_point.z() - minZ) / occupancy_grid_msg.info.resolution;
                // Set the corresponding cell as occupied
                int index = grid_z * occupancy_grid_msg.info.width + grid_y;
                //ROS_INFO("index: %d (total = %d)", index, occupancy_grid_msg.info.width * occupancy_grid_msg.info.height);

                if(index < occupancy_grid_msg.info.width * occupancy_grid_msg.info.height){
                    // Check if the point falls within the desired y-range
                    if (cam_point.x() > 0.1 && cam_point.x() < 5) {

                        occupancy_grid_msg.data[index] = 100;
                    }
                    else
                    {
                        occupancy_grid_msg.data[index] = 0;
                    }
                }
            }
        }

        occupancy_grid_pub.publish(occupancy_grid_msg);


        delete octree;
    }

    void update_global_3d_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

        for(const auto& point : cloud->points){
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {

                if((point.x < rel_min_x || point.x > rel_max_x) || (point.y < rel_min_y || point.y > rel_max_y) || (point.z < rel_min_z || point.z > rel_max_z)){
                    //ROS_INFO("Point cloud x = %f, y = %f, z = %f", point.x, point.y, point.z);
                    continue;
                }
                
                //ROS_INFO("Point cloud x = %f, y = %f, z = %f", point.x, point.y, point.z);
                tf::Vector3 point_cloud(point.x, point.y, point.z);
                tf::Vector3 point_map = world_transform * point_cloud;
                auto x = (point_map.x() - abs_min_x) / resolution;
                auto y = (point_map.y() - abs_min_y) / resolution;
                auto z = point_map.z();
                octomap::OcTreeKey key = octree.coordToKey(octomap::point3d(x, y, z));
                octree.updateNode(key, true);
            }
        }
    }
    
    std::vector<int8_t> convert_3d_map_into_2d_occupancy_grid(){

        auto occupancy_grid = std::vector<int8_t>(grid_width * grid_height, 0);

        for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
            if (octree.isNodeOccupied(*it)) {

                octomap::point3d octo_node_center = octree.keyToCoord(it.getKey()); 
                int grid_x = static_cast<int>((octo_node_center.x() - grid_origin_x) / resolution);
                int grid_y = static_cast<int>((octo_node_center.y() - grid_origin_y) / resolution);
                unsigned int index = grid_x * grid_width + grid_y;

                if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) {
                    occupancy_grid[index] = 100;
                }
                
                ROS_INFO("grid_x = %d and grid_y = %d (x = %f and y =%f) ", grid_x, grid_y, octo_node_center.x(), octo_node_center.y());
            }
        }

        return occupancy_grid;
    }

    void transform_point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // Convert msg into a point-cloud-object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        for(auto& point : cloud->points){
            point.x *= -1;
            point.y *= -1; 
        }
        // Convert the pcl::PointCloud back to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 transformed_cloud;
        pcl::toROSMsg(*cloud, transformed_cloud);
        transformed_cloud.header = msg->header;
        transformed_point_cloud_pub.publish(transformed_cloud);
    }

    void point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        
        if (!update_world_transform())
        { 
            return;
        }

        // Convert msg into a point-cloud-object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        update_global_3d_map(cloud);

        // send msg
        octomap_msgs::Octomap map_msg;
        octomap_msgs::fullMapToMsg(octree, map_msg);
        map_msg.header.frame_id = "world";
        octomap_pub.publish(map_msg);

        auto occupancy_grid = convert_3d_map_into_2d_occupancy_grid();
        nav_msgs::OccupancyGrid occupancy_grid_msg;
        occupancy_grid_msg.header.frame_id = "world";
        occupancy_grid_msg.header.stamp = ros::Time::now();
        occupancy_grid_msg.info.resolution = resolution;
        occupancy_grid_msg.info.width = grid_width;
        occupancy_grid_msg.info.height = grid_height;
        occupancy_grid_msg.info.origin.position.x = grid_origin_x;
        occupancy_grid_msg.info.origin.position.y = grid_origin_y;
        occupancy_grid_msg.info.origin.position.z = 0;
        occupancy_grid_msg.info.origin.orientation.w = 1;
        occupancy_grid_msg.data = occupancy_grid;
        occupancy_grid_pub.publish(occupancy_grid_msg);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "global_mapping_node");
    ROS_INFO_NAMED("global mapping", "started!");
    global_mapping_node n;
    ros::spin();
}
