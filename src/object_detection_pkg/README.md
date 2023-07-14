# Object_detection_pkg

Library:
-	sudo apt-get install python3-pcl
-	sudo apt-get install python-opencv

Type of messages:
-	Traffic light color: String msg 
-	Traffic light depth position topic: Float32 msg 

Nodes
-	rosrun object_detection_pkg cars_detection.py
-	rosrun object_detection_pkg traffic_light_color.py
-	rosrun object_detection_pkg traffic_light_depth.py

Topics
-	rostopic echo traffic_light_color
-	rostopic echo traffic_light_position
-	rostopic echo position_of_cars 

