## System requirements
- Ubuntu 20.04.6 LTS (64-bit)

## Tools installation
- Install basic Tools
```
sudo apt install build-essential
sudo apt install git
sudo apt-get install terminator
```
- Install ROS (see also http://wiki.ros.org/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
sudo apt update
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-noetic-desktop-full
sudo apt install python3-catkin-tools
```

## Getting Started
1. Clone the repository
2. Build project
```
cd autonomous_driving
catkin build
```
3. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
4. Unzip the Unity file and copy the files to .../devel/lib/simulation/
5. Run a test:
  - Manually control the car with w-a-s-d:
```
chmod +x ./devel/lib/simulation/Car_build.x86_64
roslaunch simulation simulation.launch
```
  - Run controller_node (i.e. The car will start driving and bump into the wall on the right) 
```
rosrun controller_pkg controller_node
```

## Code editor 
- Install vs-Code
```
sudo apt install code
```
- Open 'autonomous_driving' folder in vs-code
- In vs-code go to the Extensions sidebar (Ctrl+Shift+X). Search for "ROS" in the search bar and choose the "ROS" extension developed by Microsoft.

# Tips

Here are a couple of hints regarding the implementation. The hints are just suggestions; you are free so solve the task differently:
- The controller in controller_pkg courrently sends fixed values to the car. Start working here.
- Generating point cloud from depth image: use depth_image_proc in http://wiki.ros.org/depth_
image_proc.
- Generating occupancy Grid: use Octomap in http://wiki.ros.org/octomap.
- Please ping us in case you have any questions or if you get stuck in some subtasks.
- Use a global map as your voxel grid representation. Use a smart resolution for your voxel grid representation (e.g. 1m).



# Useful links and sources
