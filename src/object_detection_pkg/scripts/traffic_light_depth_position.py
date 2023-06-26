import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Float32

# Global variables to store the segmentation, RGB, and depth images
segmentation_image = None
rgb_image_rgb = None
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

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function to process the received RGB image message
def rgb_callback(rgb_msg):
    global rgb_image_rgb
    bridge = CvBridge()
    try:
        # Convert the received RGB image message to OpenCV format
        rgb_image_bgr = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="passthrough")
        rgb_image_rgb = cv2.cvtColor(rgb_image_bgr, cv2.COLOR_BGR2RGB)
        # Increase the resolution of the RGB image
        rgb_image_rgb = cv2.resize(rgb_image_rgb, (new_width, new_height))

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function to process the received depth image message
def depth_callback(depth_msg):
    global depth_image
    bridge = CvBridge()
    try:
        # Convert the received depth image message to OpenCV format
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Check if the segmentation and RGB images are available
        if segmentation_image is not None and rgb_image_rgb is not None:
            # Threshold the semantic segmentation image to create a binary mask
            aqua_color = np.array([0, 255, 255])
            threshold = 200  # Adjust this threshold value to segment your object accurately
            mask = np.linalg.norm(segmentation_image - aqua_color, axis=2) < threshold
            mask = mask.astype(np.uint8) * 255

            # Find contours in the binary mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Select the contour that corresponds to your object
            selected_contour = [contour for contour in contours if cv2.contourArea(contour) > 50][0]

            # Create a mask with the same shape as the RGB image and fill the contour region
            new_mask = np.zeros_like(rgb_image_rgb)
            cv2.drawContours(new_mask, [selected_contour], 0, (255, 255, 255), cv2.FILLED)

            # Apply the new mask to the RGB image to crop out the region of your object
            cropped_image_rgb = cv2.bitwise_and(rgb_image_rgb, new_mask)

            # Convert the cropped image back to BGR format
            cropped_image_bgr = cv2.cvtColor(cropped_image_rgb, cv2.COLOR_RGB2BGR)

            # Find the bounding box coordinates of the contour
            x, y, w, h = cv2.boundingRect(selected_contour)

            # Draw the bounding box on the original RGB image
            bounding_box_image = rgb_image_rgb.copy()
            cv2.rectangle(bounding_box_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Crop out the region within the bounding box
            cropped_image_bgr = cropped_image_bgr[y:y+h, x:x+w]

            # Calculate the center point of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2
            original_center_x = int(center_x * 320 / new_width)
            original_center_y = int(center_y * 240 / new_height)

            depth = depth_image[original_center_y, original_center_x]
            print("depth:", depth)

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
rgb_topic = "/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw"
depth_topic = "/unity_ros/OurCar/Sensors/DepthCamera/image_raw"
rospy.Subscriber(segmentation_topic, Image, segmentation_callback)
rospy.Subscriber(rgb_topic, Image, rgb_callback)
rospy.Subscriber(depth_topic, Image, depth_callback)

# Spin ROS
rospy.spin()

