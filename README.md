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
- Update the .bashrc file to source the ros setup file
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Getting Started
- Open a new terminal
- Clone the repository
- Build project
```
cd autonomous_driving
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```
- Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
- Unzip the Unity file and copy the files to .../devel/lib/simulation/
- Run simulation to manually control the car with w-a-s-d:
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

# Useful links and sources
