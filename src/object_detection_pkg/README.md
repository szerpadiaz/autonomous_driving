# Object_detection_pkg
first option-
roscore and run the nodes with to see the result:
-	rosrun object_detection_pkg cars_detection.py
-	rosrun object_detection_pkg traffic_light_color.py
-	rosrun object_detection_pkg traffic_light_depth.py

second option-
roslaunch simulation simulation.launch.
then echo the topic with 
-	rostopic echo traffic_light_color
-	rostopic echo traffic_light_position
-	rostopic echo position_of_cars

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

Questions:
- How to reduce the time latency / difference between simulation and node for traffic light color? For red color for example, 1 second delay

