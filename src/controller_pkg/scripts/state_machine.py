#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from object_detection_pkg.msg import DepthAndColor
from std_msgs.msg import Bool

class TrafficLightBrakingNode:
    def __init__(self):
        rospy.init_node('traffic_light_braking_node', anonymous=True)
        rospy.Subscriber('/traffic_light_status', DepthAndColor, self.traffic_light_status_callback)
        rospy.Subscriber('/true_pose', PoseStamped, self.car_position_callback)
        self.brake_active = False
        self.current_light_color = ''
        self.current_light_position = 0.0
        self.current_position_car_x = 0.0
        self.current_position_car_y= 0.0
        self.brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)

    def traffic_light_status_callback(self, msg):
        self.current_light_color = msg.dominant_color
        self.current_light_position = msg.depth

        # Check the position and determine if braking is necessary'
        if self.current_light_color == "Red" and self.current_light_position <= 23000.0:
            if self.current_light_position <= 13000.0:
            #Red light detected, activate braking
                self.brake_active = True

            if (12.0 < self.current_position_car_x < 17.0) and (-68.0 < self.current_position_car_y < -58.0):
                #rospy.loginfo( f'X: {self.current_position_car_x}, Y: {self.current_position_car_y}')
                #rospy.loginfo("First Intersection")
                self.brake_active = True
            
            if (-18 < self.current_position_car_x < -13) and (-68.0 < self.current_position_car_y < -58.0):
                #rospy.loginfo( f'X: {self.current_position_car_x}, Y: {self.current_position_car_y}')
                #rospy.loginfo("last intersection")
                self.brake_active = True

        else:
            # No red light detected, deactivate braking
            self.brake_active = False


    def car_position_callback(self, msg):
        #rospy.loginfo("car_position_callback called")
        self.current_position_car_x = msg.pose.position.x
        self.current_position_car_y = msg.pose.position.y
        #rospy.loginfo( f'X: {self.current_position_car_x}, Y: {self.current_position_car_y}')



    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Perform braking or other actions based on the state
            if self.brake_active:
                # Apply brakes
                self.apply_brakes()
            else:
                # Continue normal operation
                self.continue_driving()

            rate.sleep()

    def apply_brakes(self):
        rospy.loginfo("Applying brakes...")

        # Brake command: velocity set to 0
        bool_msg = Bool()
        bool_msg.data = True

        # Publish the brake command
        self.brake_bool.publish(bool_msg)


    def continue_driving(self):
        rospy.loginfo("Continuing driving...")

        # Add your normal driving logic here
        pass

if __name__ == '__main__':
    try:
        node = TrafficLightBrakingNode()
        node.run()
    except rospy.ROSInterruptException:
       pass
