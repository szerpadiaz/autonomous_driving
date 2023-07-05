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

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


# import sys
# # sys.path.append('/home/damvancuong/project/src/object_detection_pkg/scripts/PointCloud')
# import sys
# sys.path.append('/path/to/parent/folder/of/src')
# from src.object_detection_pkg.scripts import PointCloud as pointcl




original_center_x = 0
original_center_y = 0
contours = []

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
        pcl_cloud = convert_contours_to_pcl_cloud(contours, depth_image)
        
        # Convert pcl.PointCloud to sensor_msgs.PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'true_body'
        point_cloud_msg = create_cloud_xyz32(header, pcl_cloud.to_array())

        # fields = [
        #     pcl2.create_field('x', 0, pcl2.PointField.FLOAT32, 1),
        #     pcl2.create_field('y', 4, pcl2.PointField.FLOAT32, 1),
        #     pcl2.create_field('z', 8, pcl2.PointField.FLOAT32, 1)
        # ]

        # point_cloud_msg = pcl2.create_cloud(header, fields, points)
        point_cloud_publisher.publish(point_cloud_msg)

        # Retrieve the depth value of the current pixel
        depth = depth_image[original_center_y, original_center_x]

        # x,z is the real 2D position of the car (as (x,y) coordinate) ; y is just the height of the car
        z = depth
        x = z*29/120
        y = z/60

        # Create a Point message with the x, y, and z coordinates
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z
        #print("x", point_msg.x)

        # Publish the Point message
        point_publisher.publish(point_msg)
        print("Pixel 3D position", x, y, z)
        
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

def convert_contours_to_pcl_cloud(contours, depth_image):
    # Create a list to store the points
    points = []

    # Iterate over the contours and convert each point to 3D
    for contour in contours:
        for point in contour:
            x = point[0][0]
            y = point[0][1]
            depth = depth_image[y, x]
            z_3d = depth
            x_3d = z_3d * 29 / 120
            y_3d = z_3d / 60
            points.append([x_3d, y_3d, z_3d])

    # Create point cloud object
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(points)

    return pcl_cloud



def create_cloud(header, fields, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message.

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param fields: The point cloud fields.
    @type  fields: iterable of L{sensor_msgs.msg.PointField}
    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                elements of each iterable being the values of the fields for
                that point (in the same order as the fields parameter)
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                    height=1,
                    width=len(points),
                    is_dense=False,
                    is_bigendian=False,
                    fields=fields,
                    point_step=cloud_struct.size,
                    row_step=cloud_struct.size * len(points),
                    data=buff.raw)

def create_cloud_xyz32(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    return create_cloud(header, fields, points)

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


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