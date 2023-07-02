
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math

# Define the class for the trajectory predictor
class TrajectoryPredictor:
    def __init__(self):
        self.path_subscriber = rospy.Subscriber('path_for_trajectory', Path, self.path_callback)
        self.prediction_publisher = rospy.Publisher('/trajectory', Path, queue_size=10)

    def path_callback(self, path_msg):
        # Extract the poses from the received path message
        poses = path_msg.poses

        # Perform trajectory prediction for each pose
        predicted_path = Path()
        predicted_path.header = path_msg.header

        for i in range(len(poses)):
            current_pose = poses[i].pose

            # Perform state estimation with CTRA model
            predicted_pose = self.predict_pose(current_pose, i)

            # Create a new PoseStamped message with the predicted pose
            predicted_pose_stamped = PoseStamped()
            predicted_pose_stamped.header = path_msg.header
            predicted_pose_stamped.pose = predicted_pose

            # Add the predicted pose to the path
            predicted_path.poses.append(predicted_pose_stamped)

        # Publish the predicted path
        self.prediction_publisher.publish(predicted_path)

    def predict_pose(self, current_pose, timestamp):
        # Perform state estimation using CTRA model and other knowledge-based methods

        # Constants for CTRA model
        dt = 0.1  # Time step between each timestamp
        initial_velocity = 1.0  # Initial velocity
        initial_yaw_rate = 0.0  # Initial yaw rate
        acceleration = 0.2  # Constant acceleration
        yaw_rate = math.radians(10.0)  # Constant yaw rate

        # Predict the position based on CTRA model
        predicted_pose = current_pose

        # Compute predicted position based on CTRA model
        delta_t = dt * timestamp
        predicted_pose.position.x = current_pose.position.x + initial_velocity * math.cos(current_pose.orientation.z) * delta_t
        predicted_pose.position.y = current_pose.position.y + initial_velocity * math.sin(current_pose.orientation.z) * delta_t

        # Compute predicted yaw based on CTRA model
        predicted_yaw = current_pose.orientation.z + initial_yaw_rate * delta_t

        # Update the orientation in the predicted pose
        predicted_pose.orientation.z = predicted_yaw

        # Update the velocity and yaw rate for the next iteration
        initial_velocity += acceleration * delta_t
        initial_yaw_rate = yaw_rate

        return predicted_pose


if __name__ == '__main__':
    rospy.init_node('trajectory_predictor')
    predictor = TrajectoryPredictor()
    rospy.spin()
