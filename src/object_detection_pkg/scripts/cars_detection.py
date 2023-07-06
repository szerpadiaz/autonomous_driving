import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point
import pcl
from std_msgs.msg import Header
import pcl_ros
from collections import namedtuple
import ctypes
import math
import struct
import sys

import roslib.message
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2


original_center_x = 0
original_center_y = 0
contours = []
new_width = 1280  # Adjust the width as desired
new_height = 720  # Adjust the height as desired

# Callback function to process the received messages
def segmentation_callback(segmentation_msg):
    global original_center_x, original_center_y, contours
    # Create a publisher to publish the received messages
    bridge = CvBridge()
    try:
        # Convert the received segmentation, RGB, and depth image messages to OpenCV format
        segmentation_image = bridge.imgmsg_to_cv2(segmentation_msg, desired_encoding="passthrough")

        # Increase the resolution of the segmentation image
        segmentation_image = cv2.resize(segmentation_image, (new_width, new_height))

        # Define the blue color range for thresholding
        lower_blue = np.array([100, 0, 0], dtype=np.uint8)
        upper_blue = np.array([255, 50, 50], dtype=np.uint8)

        # Threshold the image to create a binary mask for the blue objects
        blue_mask = cv2.inRange(segmentation_image, lower_blue, upper_blue)

        # Find contours in the binary mask
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours are found
        if len(contours) != 0:
            # Process each contour individually
            for contour in contours:
                # Calculate the bounding box coordinates of the contour
                x, y, w, h = cv2.boundingRect(contour)

                # Draw the bounding box on the original image
                bounding_box_image = segmentation_image.copy()
                cv2.rectangle(bounding_box_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate the center point of the bounding box
                center_x = x + w // 2
                center_y = y + h // 2
                original_center_x = int(center_x * 320 / new_width)
                original_center_y = int(center_y * 240 / new_height)
                # print("Center Point (x, y):", original_center_x, original_center_y)
        else:
            original_center_x=0
            original_center_y=0
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return


def depth_callback(depth_msg):
    global contours
    # Create a publisher to publish the received messages
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        if len(contours) != 0:
            points = convert_contours_to_points(contours, depth_image)
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'true_body'
            point_cloud_msg = pcl2.create_cloud_xyz32(header, points)
            point_cloud_publisher.publish(point_cloud_msg)

        # Retrieve the depth value of the current pixel
        depth = depth_image[original_center_y, original_center_x]

        # x,z is the real 2D position of the car (as (x,y) coordinate) ; y is just the height of the car
        # z = depth
        # x = z*29/120
        # y = z/60
        y = depth
        x = y*29/120
        z = y/60
        # Create a Point message with the x, y, and z coordinates
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z
        #print("x", point_msg.x)

        # Publish the Point message
        point_publisher.publish(point_msg)
        #print("Pixel 3D position", x, y, z)
        
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

def convert_contours_to_points(contours, depth_image):
    points = []
    # Iterate over the contours and convert each point to 3D
    for contour in contours:
        for point in contour:
            u = int(point[0][0] * 320 / new_width)
            v = int(point[0][1] * 240 / new_height)
            depth = depth_image[v, u]

            # convert from pixel coordinates to 3d-point
            K = np.array([[120, 0, 160],
              [0, 120, 120],
              [0, 0, 1]])
            K_inv = np.linalg.inv(K)
            uv = np.array([u, v, 1])
            xyz =  K_inv @ uv
            xyz *= depth

            # convert from milimeters to meters
            x = xyz[0] / 1000
            y= xyz[1] / 1000
            z= xyz[2] / 1000
            
            if(x != 0 or y != 0 or z != 0):
                max_range_m = 25
                if(z < max_range_m):
                    #print("Point: ", x_3d, z_3d, y_3d)    
                    # append point ( z and y coordinates should be aligned with car frame)
                    points.append([x, z, y])
    return points

# Initialize the ROS node
rospy.init_node("cars_detection")

# Create a publisher to publish the received messages
point_publisher = rospy.Publisher("position_of_cars", Point, queue_size=5)
point_cloud_publisher = rospy.Publisher("point_cloud_position_of_cars", PointCloud2, queue_size=5)
# Subscribe to the segmentation, RGB, and depth image topics
segmentation_topic = "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)


# Spin ROS
rospy.spin()