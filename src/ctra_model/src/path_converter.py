#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PoseStampedToPathConverter:
    def __init__(self):
        rospy.init_node('pose_stamped_to_path_node', anonymous=True)
        self.path_pub = rospy.Publisher('path_for_trajectory', Path, queue_size=10)
        rospy.Subscriber('/unity_ros/OurCar/Sensors/IMU/pose', PoseStamped, self.pose_stamped_callback)

        self.path_message = Path()
        self.path_message.header.stamp = rospy.Time.now()
        self.path_message.header.frame_id = "path_frame"

    def pose_stamped_callback(self, pose_stamped):
        self.path_message.poses.append(pose_stamped)
        self.path_pub.publish(self.path_message)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = PoseStampedToPathConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass


