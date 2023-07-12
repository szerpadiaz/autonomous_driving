#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion

import numpy as np
import math
from geometry_msgs.msg import Quaternion
import tf

def pose_callback(pose_msg_stamped):
    # Process the received PoseStamped message and update the Odometry message
    pose_msg = pose_msg_stamped.pose
    odom_msg.pose.pose.position = pose_msg.position
    odom_msg.pose.pose.orientation = pose_msg.orientation


# Convert Quaternion message to Eigen::Quaterniond
def quaternion_msg_to_euler(quaternion_msg):
    quaternion = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler

def twist_callback(twist_msg_stamped):
    # Process the received TwistStamped message and update the Odometry message
    twist_msg = twist_msg_stamped.twist
    #odom_msg.twist.twist.linear.x = -1.0 * twist_msg.linear.x
    #odom_msg.twist.twist.linear.y = -1.0 * twist_msg.linear.y
    #odom_msg.twist.twist.linear.z = 1.0 * twist_msg.linear.z

    #euler = quaternion_msg_to_euler(odom_msg.pose.pose.orientation)
    #R = tf.transformations.euler_matrix(euler[0], euler[1], euler[2])
    #angular_vel = np.array([twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z])
    #rotated_angular_vel =  np.dot(R[:3, :3].transpose(), angular_vel)
    #odom_msg.twist.twist.angular.x = rotated_angular_vel[0]
    #odom_msg.twist.twist.angular.y = rotated_angular_vel[1]
    #odom_msg.twist.twist.angular.z = rotated_angular_vel[2]
    odom_msg.twist.twist.linear.x = 5
    odom_msg.twist.twist.linear.y = 0
    odom_msg.twist.twist.linear.z = 0
    
    # This must be corrected! It is just a cheap trick. Markus Ryll
    odom_msg.twist.twist.angular.x = 0 #twist_msg.angular.x
    odom_msg.twist.twist.angular.y = 0 #twist_msg.angular.y
    odom_msg.twist.twist.angular.z = 0 #twist_msg.angular.z


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    # Create a publisher for the odometry topic
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

    # Create an Odometry message object
    odom_msg = Odometry()

    # Set the frame ID
    odom_msg.header.frame_id = 'world'

    # Set the child frame ID
    odom_msg.child_frame_id = 'body'

    # Create subscribers for the PoseStamped and TwistStamped topics
    rospy.Subscriber('/true_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/true_twist', TwistStamped, twist_callback)

    rate = rospy.Rate(50)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Publish the Odometry message
        odom_pub.publish(odom_msg)
        rate.sleep()
