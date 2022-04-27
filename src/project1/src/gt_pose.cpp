#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "project1/parametersConfig.h"
#include <dynamic_reconfigure/server.h>
#include "nav_msgs/Odometry.h"
#include "param.h"
#include "def.h"

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

//  Setter of the struct pose
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


// POSE CLASS:
// This class manages the retrieval of pose info from the ground truth;
class PoseClass { 

    void publishMsg_GroundTruth(pose_t msg){
        nav_msgs::Odometry gt_pose_msg;

        gt_pose_msg.header.frame_id = "odom";
        gt_pose_msg.header.stamp = ros::Time::now();
        gt_pose_msg.pose.pose.position.x = msg.posit.x;
        gt_pose_msg.pose.pose.position.y = msg.posit.y;
        gt_pose_msg.pose.pose.position.z = msg.posit.z;

        gt_pose_msg.pose.pose.orientation.x = msg.orient.x;
        gt_pose_msg.pose.pose.orientation.y = msg.orient.y;
        gt_pose_msg.pose.pose.orientation.z = msg.orient.z;
        gt_pose_msg.pose.pose.orientation.w = msg.orient.w;


        gt_pose_msg.child_frame_id = "base_link";

        gt_pose_msg.twist.twist.linear.x = 0.0;
        gt_pose_msg.twist.twist.linear.y = 0.0;
        gt_pose_msg.twist.twist.linear.z = 0.0;

        gt_pose_msg.twist.twist.angular.x = 0.0;
        gt_pose_msg.twist.twist.angular.y = 0.0;
        gt_pose_msg.twist.twist.angular.z = 0.0;

        gt_publisher.publish(gt_pose_msg);

    }

/**
 * @brief This is the function we use to print out
 * the pose retrieved by the topic /robot/pose
 * 
 * @param msg 
 */
void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    pose_t actual_msg = createStruct(msg);

    publishMsg_GroundTruth(actual_msg);

    // enter this if at the first msg received, so that we set get the initial pose
    if(isFirstMsg){
        
        // * Here we can set the initial_pose dynamically
        ros::param::set("/ground_truth_listener/xPos", actual_msg.posit.x);
        ros::param::set("/ground_truth_listener/yPos", actual_msg.posit.y);
        ros::param::set("/ground_truth_listener/zPos", actual_msg.posit.z);
        
        ros::param::set("/ground_truth_listener/xOrient", actual_msg.orient.x);
        ros::param::set("/ground_truth_listener/yOrient", actual_msg.orient.y);
        ros::param::set("/ground_truth_listener/zOrient", actual_msg.orient.z);
        ros::param::set("/ground_truth_listener/wOrient", actual_msg.orient.w);

        isFirstMsg = false;
    }
    if(DEBUG){
        ROS_INFO("Message number %d arrived", actual_msg.seq);
        ROS_INFO("Time of the message: %f", actual_msg.time);
        printPosition(actual_msg.posit);
        printOrientation(actual_msg.orient);
        std::cout << std::endl;

    }



}

public:
    // The class subscribes to the ground truth  topic /robot/pose
    PoseClass() {
        gt_publisher = nh.advertise<nav_msgs::Odometry>("/gtpose", 1000);
        subPose = nh.subscribe("robot/pose", 1000, &PoseClass::poseCallBack, this);
    }



private:
    ros::NodeHandle nh;
    ros::Subscriber subPose;
    ros::Publisher gt_publisher;

    bool isFirstMsg = true;

};

void param_callback(double* xPos, double* yPos, double* zPos,
                    double* xOrient, double* yOrient, double* zOrient, double* wOrient,
                    project1::parametersConfig &config, uint32_t level){

    
    ROS_INFO("Reconfigure request, new values are: %f", *xPos);

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ground_truth_listener");

    pose_t initPose;
    
    dynamic_reconfigure::Server<project1::parametersConfig> dynServ;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

    f = boost::bind(&param_callback, &initPose.posit.x, &initPose.posit.y, &initPose.posit.z, 
                                    &initPose.orient.x, &initPose.orient.y,&initPose.orient.z, &initPose.orient.w, _1, _2); 
    dynServ.setCallback(f);

    PoseClass p;

    ros::spin();

    return 0;
}
