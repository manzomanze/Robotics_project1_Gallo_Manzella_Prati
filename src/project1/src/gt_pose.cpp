#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

// these structs are used to encapsule in a cleaner way the message
// which will be listened on the topic
typedef struct {
    int x;
    int y;
    int z;
} position_t;

typedef struct {
    int x;
    int y;
    int z;
    int w;
} orientation_t;


typedef struct {
    uint32_t seq;
    double time;

    position_t posit;
    orientation_t orient;  
} t_pose;

void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {



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
