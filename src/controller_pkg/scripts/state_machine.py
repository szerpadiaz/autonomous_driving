#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class TrafficLightBrakingNode:
    def __init__(self):
        rospy.init_node('traffic_light_braking_node', anonymous=True)
        rospy.Subscriber('/traffic_light_color', String, self.traffic_light_color_callback)
        rospy.Subscriber('/traffic_light_position', Float32, self.traffic_light_position_callback)
        self.brake_active = False
        self.current_light_color = ''
        self.current_light_position = 0.0
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel2', Twist, queue_size=10)

    def traffic_light_color_callback(self, msg):
        self.current_light_color = msg.data

    def traffic_light_position_callback(self, msg):
        self.current_light_position = msg.data
        # Check the position and determine if braking is necessary
        if self.current_light_color == 'Red' and self.current_light_position <= 19000.0:
            # Red light detected, activate braking
            self.brake_active = True
        else:
            # No red light detected, deactivate braking
            self.brake_active = False

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
        brake_cmd = Twist()
        brake_cmd.linear.x = 0.0
        brake_cmd.angular.z = 0.0

        # Publish the brake command
        self.cmd_vel_pub.publish(brake_cmd)

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
