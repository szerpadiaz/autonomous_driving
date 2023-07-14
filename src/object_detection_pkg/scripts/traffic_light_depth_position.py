import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32


# Global variables to store the segmentation, RGB, and depth images
segmentation_image = None
depth_image = None

# Callback function to process the received segmentation message
def segmentation_callback(segmentation_msg):
    global segmentation_image
    bridge = CvBridge()
    try:
        # Convert the received segmentation image message to OpenCV format
        segmentation_image = bridge.imgmsg_to_cv2(segmentation_msg, desired_encoding="passthrough")
        # Convert the segmentation image to RGB
        segmentation_image = cv2.cvtColor(segmentation_image, cv2.COLOR_BGR2RGB)
        # Increase the resolution of the segmentation image
        segmentation_image = cv2.resize(segmentation_image, (new_width, new_height))
        #take the middle part of the image only to detect traffic light
        segmentation_image = segmentation_image[:,360:720]
        

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function to process the received depth image message
def depth_callback(depth_msg):
    global depth_image
    bridge = CvBridge()
    try:
        # Convert the received depth image message to OpenCV format
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Check if the segmentation are available
        if segmentation_image is not None:
            # Threshold the semantic segmentation image to create a binary mask
            aqua_color = np.array([0, 255, 255])
            threshold = 200  # Adjust this threshold value to segment your object accurately
            mask = np.linalg.norm(segmentation_image - aqua_color, axis=2) < threshold
            mask = mask.astype(np.uint8) * 255
            # Find contours in the binary mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # Select the contour that corresponds to the object
                selected_contour = max(contours, key=cv2.contourArea)
                # Check if the contour area is greater than 50
                if cv2.contourArea(selected_contour) > 50:
                    # Find the bounding box coordinates of the contour
                    x, y, w, h = cv2.boundingRect(selected_contour)

                    # Calculate the center point of the bounding box
                    center_x = x +360 + w // 2
                    center_y = y + h // 2

                    #convert position b
                    original_center_x = int(center_x * 320 / new_width)
                    original_center_y = int(center_y * 240 / new_height)

                    depth = depth_image[original_center_y, original_center_x]   #depth value here is in mm

                    # Publish the depth value
                    depth_publisher.publish(depth)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Initialize the ROS node
rospy.init_node("traffic_light_position")

# Create a publisher to publish the depth value
depth_publisher = rospy.Publisher("traffic_light_position", Float32, queue_size=10)

# Set the desired resolution for the images
new_width = 1280  # Adjust the width as desired
new_height = 720  # Adjust the height as desired

# Subscribe to the segmentation, RGB, and depth image topics
segmentation_topic = "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
#rospy.Subscriber(rgb_topic, Image, rgb_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)

# Spin ROS
rospy.spin()


