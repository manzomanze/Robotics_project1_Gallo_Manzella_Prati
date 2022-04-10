#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "pub_sub/Num.h"



class OdometryCalculator{
public:
  void odometryCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("ODOMETRY position [%f],[%f],[%f],[%f]",msg->position[0],msg->position[1],msg->position[2],msg->position[3]);
    ROS_INFO("ODOMETRY velocity [%f],[%f],[%f],[%f]\n",msg->velocity[0],msg->velocity[1],msg->velocity[2],msg->velocity[3]);
  }
  OdometryCalculator(){
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::odometryCallback,this);
  }
  
private:
  ros::NodeHandle n;
  ros::Subscriber sub_encoder_wheel;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  OdometryCalculator odom;
  //ros::NodeHandle n;

  //ros::Subscriber sub_encoder_wheel = n.subscribe("wheel_states", 1000, chatterCallback);

  ros::spin();

  return 0;
}
