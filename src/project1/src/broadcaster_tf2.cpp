#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "nav_msgs/Odometry.h"


class TfBroad {
public:
    


    void base_link_callback(const nav_msgs::Odometry::ConstPtr& msg){
        odom_base_link.header.stamp = ros::Time::now();
        odom_base_link.header.frame_id = "odom";
        odom_base_link.child_frame_id = "base_link";
        odom_base_link.transform.translation.x = msg->pose.pose.position.x;
        odom_base_link.transform.translation.y = msg->pose.pose.position.y;
        odom_base_link.transform.translation.z = 0.0;
        
        odom_base_link.transform.rotation = msg->pose.pose.orientation;
        br.sendTransform(odom_base_link);
    }

    TfBroad() {
        sub_odometry = n.subscribe("odom", 1000, &TfBroad::base_link_callback,this);
    }
private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped odom_base_link;
    ros::Subscriber sub_odometry;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_broadcast");
    TfBroad my_tf_broadcaster;
    ros::spin();


  return 0;
}