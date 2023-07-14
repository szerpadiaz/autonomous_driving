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
<<<<<<< HEAD
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *world* frame
=======
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  float omega_control;
  float velocity_control;
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle
  double yaw_current;

  double hz;             // frequency of the main control loop

  double Kp_v;      // Proportional gain for velocity
  double Ki_v;      // Integral gain for velocity
  double err_sum_v; // Sum of errors for velocity

  double Kp_yaw;      // Proportional gain for yaw
  double Ki_yaw;      // Integral gain for yaw
  double err_sum_yaw; // Sum of errors for yaw
  double last_time;   // The last time the control loop was executed

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
<<<<<<< HEAD
      local_path = nh.subscribe("/move_base/TrajectoryPlannerROS/local_plan", 1, &controllerNode::onLocalPath, this);
      
      // initial values for velocity control and yaw control
      Kp_v = 0.5;
      Ki_v = 0.1;
      err_sum_v = 0;
      Kp_yaw = 0.5;
      Ki_yaw = 0.1;
      err_sum_yaw = 0;
      last_time = ros::Time::now().toSec();

      // The car will start moving in the x direction with a velocity of 1.0 m/s
      vd << 0.0, 0, 0;

=======
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);
  }

  void onCmdVelocity(const geometry_msgs::Twist& msg){
    linear_vel = msg.linear.x;
    angular_vel = msg.angular.z;
<<<<<<< HEAD

    if(linear_vel < 0)
      linear_vel *= -1;
    //ROS_INFO("cmd_vel: linear_vel: %f, angular_vel: %f", linear_vel, angular_vel);

    // Set the desired linear velocity in the x direction
    vd(0) = linear_vel;

    // Set the desired yaw rate
    yawd = angular_vel;
=======
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78
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

<<<<<<< HEAD
    // Get current yaw
    tf::Quaternion tf_q;
    tf::quaternionEigenToTF(q, tf_q);
    yaw_current = tf::getYaw(tf_q);
=======
    // Update previous errors
    previous_error_v = velocity_control;
    previous_error_omega = omega_control;
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78
  }


  void controlLoop(const ros::TimerEvent& t){
    mav_msgs::Actuators msg;

    msg.angular_velocities.resize(4);

<<<<<<< HEAD
    // Calculate the acceleration command based on the velocity error
    double err_v = vd(0) - v(0);
    err_sum_v += err_v * (ros::Time::now().toSec() - last_time);
    auto acceleration = Kp_v * err_v + Ki_v * err_sum_v;

    // Calculate the turning angle rate command based on the yaw error
    double err_yaw = yawd - omega(2); // omega(2) is the current yaw rate
    err_sum_yaw += err_yaw * (ros::Time::now().toSec() - last_time);
    auto yaw_rate = Kp_yaw * err_yaw + Ki_yaw * err_sum_yaw;

    msg.angular_velocities[0] = 0; //acceleration;
    msg.angular_velocities[1] = 0; //yaw_rate;

=======
    auto acc = Kp_v * velocity_control + Ki_v * integral_error_v + Kd_v * derivative_error_v;
    auto turning_rate = Kp_omega * omega_control + Ki_omega * integral_error_omega + Kd_omega * derivative_error_omega;

    if(acc < 0) {
      acc = 0;
    }
    if(acc > 5) {
      acc = 5;
    }

    msg.angular_velocities[0] = acc;
    msg.angular_velocities[1] = turning_rate;
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78
    msg.angular_velocities[2] = 0;  // Breaking
    msg.angular_velocities[3] = 0;

    car_commands.publish(msg);
<<<<<<< HEAD

    last_time = ros::Time::now().toSec();

    ROS_INFO("acceleration: %f", acceleration);
    ROS_INFO("yaw_rate: %f", yaw_rate);
=======
    //ROS_INFO("acceleration: %f, turning-angle-rate: %f ", acc, turning_rate);
>>>>>>> ff67ebf761d7e033997a415bea1db249b85d1e78
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}