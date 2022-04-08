#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pub_sub/Num.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void counterCallback(const pub_sub::Num::ConstPtr& msg) {
  ROS_INFO("I counted: [%d]", msg->num);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub_chatter = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub_counter = n.subscribe("counter", 1000, counterCallback);

  ros::spin();

  return 0;
}
