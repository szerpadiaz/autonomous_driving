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
  float velocity_control;

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  float linear_vel;
  float angular_vel;

  double integral_error_v = 0.0;
  double derivative_error_v = 0.0; 
  double previous_error_v = 0.0;
  double derivative_error_omega = 0.0;
  double previous_error_omega = 0.0;
  double integral_error_omega = 0.0;

  double Kp_v = 1.0; //3;
  double Ki_v = 0.3; //36;
  double Kd_v = 0.01; 

  double Kp_omega = 1.0;
  double Kd_omega = 0.1;
  double Ki_omega = 0;

public:
  controllerNode():hz(50.0){
      
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      cmd_vel = nh.subscribe("cmd_vel", 1, &controllerNode::onCmdVelocity, this);
      car_commands = nh.advertise<mav_msgs::Actuators>("car_commands", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onCmdVelocity(const geometry_msgs::Twist& msg){
    linear_vel = msg.linear.x;
    angular_vel = msg.angular.z;
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
    integral_error_omega += omega_control * (1.0 / hz);
    derivative_error_omega = (omega_control - previous_error_omega) * hz;

    velocity_control = v(0) - linear_vel;
    if(velocity_control < -1) {
      velocity_control = -1;
    }
    if(velocity_control > 4) {
      velocity_control = 4;
    }
    integral_error_v += velocity_control * (1.0 / hz);
    derivative_error_v = (velocity_control - previous_error_v) * hz;

    // Update previous errors
    previous_error_v = velocity_control;
    previous_error_omega = omega_control;
  }


  void controlLoop(const ros::TimerEvent& t){

    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);

    auto acc = Kp_v * velocity_control + Ki_v * integral_error_v + Kd_v * derivative_error_v;
    auto turning_rate = Kp_omega * omega_control + Ki_omega * integral_error_omega + Kd_omega * derivative_error_omega;

    msg.angular_velocities[0] = acc;
    msg.angular_velocities[1] = turning_rate;
    msg.angular_velocities[2] = 0;  // Breaking
    msg.angular_velocities[3] = 0;

    car_commands.publish(msg);
    //ROS_INFO("acceleration: %f, turning-angle-rate: %f ", acc, turning_rate);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
