#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities
#include "pub_sub/Num.h"



class OdometryCalculator{
public:
  //callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    ROS_INFO("ODOMETRY position [%f],[%f],[%f],[%f]",msg->position[0],msg->position[1],msg->position[2],msg->position[3]);
    ROS_INFO("ODOMETRY velocity [%f],[%f],[%f],[%f]\n",msg->velocity[0],msg->velocity[1],msg->velocity[2],msg->velocity[3]);
    
  }

  //constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::encoderCallback,this);
    //linear_anglular_velocities = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }
  
private:
  ros::NodeHandle n;
  ros::Subscriber sub_encoder_wheel;
  ros::Publisher linear_angular_velocities;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  //odom object of class OdometryCalcualtor
  OdometryCalculator odom; 

  ros::spin();

  return 0;
}
