# Object_detection_pkg

library : sudo apt-get install python3-pcl
	sudo apt-get install python-opencv


run the nodes with to see the result (roscore first):
-	rosrun object_detection_pkg cars_detection.py
-	rosrun object_detection_pkg traffic_light_color.py
-	rosrun object_detection_pkg traffic_light_depth_position.py


can also see the results through roslaunch simulation simulation.launch.
then echo the topic with 
-	rostopic echo traffic_light_color
-	rostopic echo traffic_light_position
-	rostopic echo position_of_cars
-	rostopic echo point_cloud_position_of_cars

Type and how see/access the topic msg like below:
-	Traffic light color: String msg 
-	Traffic light depth position topic: Float32 msg 
-	Car position topic : Point msg (3D coordinate) 
-	(the trafic light color doesn't working properly yet and will be updated then)



nodes
rosrun object_detection_pkg cars_detection.py
rosrun object_detection_pkg traffic_light_color.py
rosrun object_detection_pkg traffic_light_depth.py

topics
rostopic echo traffic_light_color
rostopic echo traffic_light_position
rostopic echo position_of_cars (x,z is the position of the car in 2D map; y is just the height of the car)

Add 3 below lines to integrate 3 nodes to the simulation.launch
-	<node name="cars_detection_node" pkg="object_detection_pkg" type="cars_detection.py" />
-	<node name="traffic_light_color_node" pkg="object_detection_pkg" type="traffic_light_color.py" />
-	<node name="traffic_light_position_node" pkg="object_detection_pkg" type="traffic_light_depth_position.py" />

Questions:
- How to reduce the time latency / difference between simulation and node for traffic light color? For red color for example, 1 second delay

