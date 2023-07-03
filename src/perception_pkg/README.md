# Getting started
* Install following packages
```
sudo apt-get install ros-noetic-depth-image-proc
sudo apt-get install ros-noetic-octomap
sudo apt-get install ros-noetic-octovis
sudo apt-get install ros-noetic-octomap-msgs
sudo apt-get install ros-noetic-octomap-rviz-*
sudo apt-get install ros-noetic-octomap-mapping

sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-pcl-conversions
sudo apt-get install python3-pcl

```
* Launch simulation and perception
```
roslaunch simulation simulation.launch
roslaunch perception_pkg perception.launch
```

# Implementation 

Generating point cloud from depth image using depth_image_proc.
(See http://wiki.ros.org/depth_image_proc) 

The depth_image_proc is a ROS package subscribes to a depth image topic, then process the images to generate a point cloud.
The package can be launch using nodelet with the following parameters:
- Topic for camera info: "/unity_ros/OurCar/Sensors/DepthCamera/camera_info"
- Topic for raw depth image: "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
- Topic for point cloud: "points_cloud"
- Function to be used for processing: "depth_image_proc/point_cloud_xyz"
See also perception.launch

To visualize the point cloud, we can use rviz. A configuration file was created as follows:
- Open RVIZ
- Add a new display of type "PointCloud2"
- In the "PointCloud2", set the "Topic" to "points_cloud"
- In the "Global Options", set "Fixed Frame" to "/OurCar/Sensors/DepthCamera" (the coordinate frame that the point_cloud is referenced to)
See also cfg/point_cloud.rviz


The depth_image_proc package uses the metadata in the image message header to determine the depth encoding and interpret the depth values correctly. It applies depth processing algorithms to the input depth image and publishes the resulting point cloud on a new topic.

Therefore, by providing a depth image topic to the depth_image_proc package, it understands that the input data is a depth image and processes it accordingly to generate a point cloud. It handles the depth-specific computations internally based on the provided topic, assuming the depth information is present in the image message data.

- Generating occupancy Grid: using Octomap.
(Use https://wiki.ros.org/octomap, https://wiki.ros.org/octovis)

- Build global map from the voxel grid representation (voxel grid representation 1m).
