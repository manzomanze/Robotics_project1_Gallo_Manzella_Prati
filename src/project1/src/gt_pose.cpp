#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

// these structs are used to encapsule in a cleaner way the message
// which will be listened on the topic
typedef struct {
    double x;
    double y;
    double z;
} position_t;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} orientation_t;

typedef struct {
    uint32_t seq;
    double time;

    position_t posit;
    orientation_t orient;  
} pose_t;

void printPosition(position_t position) {
    ROS_INFO("POSITION  x: %f, y: %f, z: %f", position.x, position.y, position.z);
    
}

void printOrientation(orientation_t orient) {
    ROS_INFO("ORIENTATION  x: %f, y: %f, z: %f, w: %f", orient.x, orient.y, orient.z, orient.w);
    
}

pose_t createStruct(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_t p;
    
    p.seq = msg -> header.seq;
    p.time = msg -> header.stamp.toSec();
    
    p.posit.x = msg -> pose.position.x;
    p.posit.y = msg -> pose.position.y;
    p.posit.z = msg -> pose.position.z;

    p.orient.x = msg -> pose.orientation.x;
    p.orient.y = msg -> pose.orientation.y;
    p.orient.z = msg -> pose.orientation.z;
    p.orient.w = msg -> pose.orientation.w;
    
    return p;

}


/**
 * @brief This is the function we use to print out
 * the pose retrieved by the topic /robot/pose
 * 
 * @param msg 
 */
void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    pose_t actual_msg = createStruct(msg);

    ROS_INFO("Message number %d arrived", actual_msg.seq);
    ROS_INFO("Time of the message: %f", actual_msg.time);
    printPosition(actual_msg.posit);
    printOrientation(actual_msg.orient);
    std::cout << std::endl;


}

class poseClass {

public:

    poseClass() {
        subPose = nh.subscribe("robot/pose", 1000, poseCallBack);
    }



private:
    ros::NodeHandle nh;
    
    ros::Subscriber subPose;

};



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ground_truth_listener");
    
    poseClass posClass;

    ros::spin();

    return 0;
}