#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#define PI M_PI


#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>

class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber current_state;
  ros::Publisher car_commands;
  ros::Subscriber cmd_vel;
  ros::Subscriber local_path;

  ros::Timer timer;


  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  float omega_control;

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  float linear_vel;
  float angular_vel;

public:
  controllerNode():hz(50.0){
      
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      cmd_vel = nh.subscribe("cmd_vel", 1, &controllerNode::onCmdVelocity, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      local_path = nh.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, &controllerNode::onLocalPath, this);
      
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onLocalPath(const nav_msgs::Path::ConstPtr& localPlan){
    int numPoses = localPlan->poses.size();
    //ROS_INFO("Number of poses in the local plan: %d", numPoses);

    for (const auto& pose : localPlan->poses)
    {
      const auto& position = pose.pose.position;
      const auto& orientation = pose.pose.orientation;
      //ROS_INFO("Position: [x=%.2f, y=%.2f, z=%.2f]", position.x, position.y, position.z);
      //ROS_INFO("Orientation: [x=%.2f, y=%.2f, z=%.2f, w=%.2f]", orientation.x, orientation.y, orientation.z, orientation.w);
    }
  }

  void onCmdVelocity(const geometry_msgs::Twist& msg){
    linear_vel = msg.linear.x;
    
    //Eigen::Vector3d omega_vector;
    //omega_vector[0] = msg.angular.x;
    //omega_vector[1] = msg.angular.y;
    //omega_vector[2] = msg.angular.z;
    //auto desired_omega = R.transpose() * omega_vector;
    //angular_vel = desired_omega[2];
    angular_vel = msg.angular.z;

    if(linear_vel < 0)
      linear_vel *= -1;
    //ROS_INFO("cmd_vel: linear_vel_x: %f, linear_vel_y: %f, linear_vel_z %f", msg.linear.x, msg.linear.y, msg.linear.z);

  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      
    x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
    v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
    R = q.toRotationMatrix();


    // Rotate omega
    omega = R.transpose() * omega;
    v = R.transpose() * v;
    omega_control  = (-angular_vel);
    if (omega_control < -3){
      omega_control = -3;}
    if (omega_control > 3){
      omega_control = 3;}
      
    
    

    ROS_INFO("Stuff: omega_control %f, angular_vel %f, omega %f", omega_control, angular_vel, omega[2]);
    //ROS_INFO("Current state - velocity: %f, %f, %f", v[0], v[1], v[2]);
  ////ROS_INFO("Current state - orientation: %f, %f, %f", R[0][0], R[1][1], R[2][2]);
    //ROS_INFO("Current state - angular-vel: %f, %f, %f", omega[0], omega[1], omega[2]);
  }


  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;

    if (linear_vel < 1.1)
      {linear_vel = 1.1;}
    if (linear_vel > 3.0)
      {linear_vel = 3.0;}
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = 4 - v(0); // Acceleration
    msg.angular_velocities[1] = omega_control;  // Turning angle rate
    msg.angular_velocities[2] = 0;  // Breaking
    msg.angular_velocities[3] = 0;
    
    //ROS_INFO("cmd_vel: linear_vel: %f, angular_vel: %f", linear_vel, omega_control);

    car_commands.publish(msg);

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
