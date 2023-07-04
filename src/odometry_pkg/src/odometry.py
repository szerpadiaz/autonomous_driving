#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion

def pose_callback(pose_msg_stamped):
    # Process the received PoseStamped message and update the Odometry message
    pose_msg = pose_msg_stamped.pose
    odom_msg.pose.pose.position = pose_msg.position
    odom_msg.pose.pose.orientation = pose_msg.orientation

def twist_callback(twist_msg_stamped):
    # Process the received TwistStamped message and update the Odometry message
    twist_msg = twist_msg_stamped.twist
    odom_msg.twist.twist.linear = twist_msg.linear
    odom_msg.twist.twist.angular = twist_msg.angular

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    # Create a publisher for the odometry topic
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # Create an Odometry message object
    odom_msg = Odometry()

    # Set the frame ID
    odom_msg.header.frame_id = 'odom'

    # Set the child frame ID
    odom_msg.child_frame_id = 'true_body'

    # Create subscribers for the PoseStamped and TwistStamped topics
    rospy.Subscriber('/unity_ros/OurCar/Sensors/IMU/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/unity_ros/OurCar/Sensors/IMU/twist', TwistStamped, twist_callback)

    rate = rospy.Rate(1000)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Publish the Odometry message
        odom_pub.publish(odom_msg)
        rate.sleep()