#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "param.h"
#include "def.h"
#include "project1/Wheels.h"

class Wheels{
  public: 
    double getWheel(wheel_order enumeration){
      return wheel_velocity[enumeration];
    }
    void setWheel(double value, wheel_order enumeration){
      wheel_velocity[enumeration] = value;
    }
    Wheels(){}

  private:
    double wheel_velocity[N_WHEELS];
};

class ComputeControl{
public:

  Wheels calculateControl(){

    double rpm_fl;
    double rpm_fr;
    double rpm_rr;
    double rpm_rl;

    rpm_fl = SEC_IN_MIN*GEAR_RATIO/RADIUS_WHEEL*(( - X_WHEEL_DISTANCE - Y_WHEEL_DISTANCE) * velocity_angular_z + velocity_linear_x - velocity_linear_y);
    rpm_fr = SEC_IN_MIN*GEAR_RATIO/RADIUS_WHEEL*((   X_WHEEL_DISTANCE + Y_WHEEL_DISTANCE) * velocity_angular_z + velocity_linear_x + velocity_linear_y);
    rpm_rr = SEC_IN_MIN*GEAR_RATIO/RADIUS_WHEEL*((   X_WHEEL_DISTANCE + Y_WHEEL_DISTANCE) * velocity_angular_z + velocity_linear_x - velocity_linear_y);
    rpm_rl = SEC_IN_MIN*GEAR_RATIO/RADIUS_WHEEL*(( - X_WHEEL_DISTANCE - Y_WHEEL_DISTANCE) * velocity_angular_z + velocity_linear_x + velocity_linear_y);

    Wheels wheel_velocities;
    wheel_velocities.setWheel(rpm_fl,FL);
    wheel_velocities.setWheel(rpm_fr,FR);
    wheel_velocities.setWheel(rpm_rr,RR);
    wheel_velocities.setWheel(rpm_rl,RL);
    return wheel_velocities;

  }

  void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    ROS_INFO("%f %f %f",msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z);
    
    velocity_linear_x = msg->twist.linear.x;
    velocity_linear_y = msg->twist.linear.y;
    velocity_angular_z = msg->twist.angular.z;

    Wheels wheel_velocities = calculateControl();

    publishMsg_wheels(wheel_velocities);
  }
  
  // Publishes robot odometry on the topic odom
  void publishMsg_wheels(Wheels wheel_vel_to_publish){
    /* generate nav_msgs::Odometry msg containing the position 
    on x and  y and the angular position around the z axis */
    project1::Wheels wheels_msg;
    
    
    wheels_msg.header.frame_id = "base_link";
    wheels_msg.header.stamp = ros::Time::now();
    
    wheels_msg.rpm_fl = wheel_vel_to_publish.getWheel(FL);
    wheels_msg.rpm_fr = wheel_vel_to_publish.getWheel(FR);
    wheels_msg.rpm_rr = wheel_vel_to_publish.getWheel(RR);
    wheels_msg.rpm_rl = wheel_vel_to_publish.getWheel(RL);

    // publish messages
    control_publisher.publish(wheels_msg);
  }

  // Constructor of the class ComputeControl
  ComputeControl(){ 
    
    
  
    cmd_vel_subscribe = n.subscribe("cmd_vel", 1000, &ComputeControl::cmdVelCallback,this);    
    control_publisher = n.advertise<project1::Wheels>("/wheels_rpm", 1000); 
  }
  
private:
  
  double velocity_linear_x;
  double velocity_linear_y;
  double velocity_angular_z;

  // robot pose (Position and Orientation) in a fixed Frame of Reference
  
  
  ros::NodeHandle n;
  ros::Subscriber cmd_vel_subscribe;
  ros::Publisher control_publisher;
  ros::ServiceServer service;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_control");
  //odom object of class OdometryCalculator
  ComputeControl control;

  ros::spin();

  return 0;
}