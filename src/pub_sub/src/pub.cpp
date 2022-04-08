#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pub_sub/Num.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker"); //initialize the node
  ros::NodeHandle n; //get a nodehandle to call the functions over this node 

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); //advertise some type of message over a topic
  ros::Publisher counter_pub = n.advertise<pub_sub::Num>("counter", 1000); //the Num message is described in the msg folder  of this package

  int count = 0;

  ros::Rate loop_rate(10); // this limits the rate of the loop at 10 hz that means it publishes messages on the topics in 10 times per second 

  while (ros::ok()) { /*while (ros::ok()) is just a better way to write while(1): it’ll handle
    interrupts and stop if a new node with the same name is created or a shutdown
    command is called */

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

    ros::spinOnce();/*ros::spin() will simply cycle as fast as possible, calling our callback
    when needed, but without using CPU if there is nothing to do*/

    loop_rate.sleep(); // if we have finished before the looprate limited  at 10 Hz then we wait for the end of the time slot

    ++count;
  }

  return 0;
}
