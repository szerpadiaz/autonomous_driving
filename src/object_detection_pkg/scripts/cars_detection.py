import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point

original_center_x = 0
original_center_y = 0

# Callback function to process the received messages
def segmentation_callback(segmentation_msg):
    global original_center_x, original_center_y
    # Create a publisher to publish the received messages
    bridge = CvBridge()
    try:
        # Convert the received segmentation, RGB, and depth image messages to OpenCV format
        segmentation_image = bridge.imgmsg_to_cv2(segmentation_msg, desired_encoding="passthrough")

        # Increase the resolution of the segmentation image
        new_width = 1280  # Adjust the width as desired
        new_height = 720  # Adjust the height as desired
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
    # Create a publisher to publish the received messages
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        # Get the camera intrinsics fu: 97.87426995358646, fv: 97.87426995358646, pu: 160.0, pv: 90.0, s: 0
        fx = 97.87426995358646  # Focal length in x direction
        fy = 97.87426995358646  # Focal length in y direction
        cx = 160.0  # Principal point x coordinate
        cy = 90.0  # Principal point y coordinate

        # Retrieve the depth value of the current pixel
        depth = depth_image[original_center_y, original_center_x]

        # Calculate the x, y, and z coordinates based on the depth value and camera intrinsics
        z = depth
        x = (original_center_x - cx) * z / fx
        y = (original_center_y - cy) * z / fy

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


# Initialize the ROS node
rospy.init_node("cars_detection")

# Create a publisher to publish the received messages
point_publisher = rospy.Publisher("position_of_cars", Point, queue_size=10)

# Subscribe to the segmentation, RGB, and depth image topics
segmentation_topic = "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)

# Spin ROS
rospy.spin()
