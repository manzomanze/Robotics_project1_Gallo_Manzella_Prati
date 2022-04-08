#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pub_sub/Num.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher counter_pub = n.advertise<pub_sub::Num>("counter", 1000);

  int count = 0;

  ros::Rate loop_rate(10);

  while (ros::ok()) {

    // generate chatter msg
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    // generate counter msg
    pub_sub::Num count_msg;
    count_msg.num = count;

    // print count to screen
    ROS_INFO("%d", count);

    // publish messages
    chatter_pub.publish(msg);
    counter_pub.publish(count_msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}
