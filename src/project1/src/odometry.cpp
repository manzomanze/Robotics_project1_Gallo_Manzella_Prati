#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "project1/ResetPose.h"
#include "param.h"
#include "def.h"


enum wheel_order {
  FL,
  FR,
  RL,
  RR
};

typedef struct t_wheel_pos_vel{
  double vel[N_WHEELS];
  double count_ticks[N_WHEELS];
} t_wheel_pos_vel;

typedef struct t_msg {
  uint32_t seq;
  double time;
  t_wheel_pos_vel wheel_info;
} t_msg;

class OdometryCalculator{
public:

  void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    ROS_INFO("%f %f %f",msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z);
    calculateRungeKuttaIntegration((double)msg->twist.linear.x,(double)msg->twist.linear.y,(double)msg->twist.angular.z,msg->header.stamp.toSec());
    publishMsg_odom(robot_x,robot_y,robot_theta);
  }
  
  // Publishes robot odometry on the topic odom
  void publishMsg_odom(double robot_x, double robot_y, double robot_theta){
    /* generate nav_msgs::Odometry msg containing the position 
    on x and  y and the angular position around the z axis */
    nav_msgs::Odometry odom_msg;
    
    
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = ros::Time::now();
    
    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    
    // Create this quaternion from roll/pitch/yaw (in radians)
    quat_tf.setRPY( 0, 0,  robot_theta); //to transform in radians if not in radians
    tf2::convert(quat_tf,quat_msg);
    odom_msg.pose.pose.orientation = quat_msg;
    // print count to screen

    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = robotLinearVelocityOnX;
    odom_msg.twist.twist.linear.y = robotLinearVelocityOnY;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = robotAngularVelocity;

    // publish messages
    odom_publisher.publish(odom_msg);
  }
  // Calculates the odometry using Euler Integration Method
  void calculateEulerIntegration(double linear_x, double linear_y, double angular_z, double samplingTime){
    double vel_kx = (linear_x* cos(robot_theta)-linear_y * sin(robot_theta));
    robot_x += vel_kx *samplingTime * cos(robot_theta) ;

    double vel_ky = (linear_x* sin(robot_theta)+linear_y * cos(robot_theta));
    robot_y += vel_ky*samplingTime * sin(robot_theta);
    
    robot_theta += angular_z * samplingTime;
  
    if(DEBUG){
      ROS_INFO("Robot X [%f] Robot Y [%f] Robot Theta [%f]",robot_x,robot_y,robot_theta);
    }
  }
  // Calculates the odometry using Runge Kutta Integration Method
  void calculateRungeKuttaIntegration(double linear_x, double linear_y, double angular_z, double samplingTime){
    double rungeKuttaAdditionalRotation = angular_z*samplingTime/2;
    double vel_kx = (linear_x* cos(robot_theta)-linear_y * sin(robot_theta+rungeKuttaAdditionalRotation));
    robot_x += vel_kx *samplingTime * cos(robot_theta) ;

    double vel_ky = (linear_x* sin(robot_theta)+linear_y * cos(robot_theta+rungeKuttaAdditionalRotation));
    robot_y += vel_ky*samplingTime;
    
    robot_theta += angular_z * samplingTime;
    if(DEBUG){
      ROS_INFO("Robot X [%f] Robot Y [%f] Robot Theta [%f]",robot_x,robot_y,robot_theta);
    }
  }


 


  bool resetPose(project1::ResetPose::Request &req, project1::ResetPose::Response &res) {
    robot_x = req.linearx;
    robot_y = req.lineary;
    robot_theta = req.angulartheta;
    ROS_INFO("Set to:%f, %f, %f", robot_x, robot_y, robot_theta);

    res.result = 200;
    return true;
  }

  // Constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    
    float test = 0.0; 

    service = n.advertiseService("resetpose", &OdometryCalculator::resetPose, this);
  
    cmd_vel_subscribe = n.subscribe("cmd_vel", 1000, &OdometryCalculator::cmdVelCallback,this);    
    odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1000); 
  }
  
private:
  


  // robot pose (Position and Orientation) in a fixed Frame of Reference
  double robot_x;
  double robot_y;
  double robot_theta;

  double robotLinearVelocityOnX;
  double robotLinearVelocityOnY;
  double robotAngularVelocity;
  
  ros::NodeHandle n;
  ros::Subscriber cmd_vel_subscribe;
  ros::Publisher odom_publisher;
  ros::ServiceServer service;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "speed_calculator");
  //odom object of class OdometryCalculator
  OdometryCalculator odom;

  ros::spin();

  return 0;
}
