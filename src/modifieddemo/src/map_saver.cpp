#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "modifieddemo/SaveMap.h"


class map_saver {

private:
	ros::NodeHandle n; 
	ros::Subscriber sub_map;
	ros::Subscriber sub_trajectory;
	ros::ServiceServer service;
	nav_msgs::OccupancyGrid map;
	nav_msgs::OccupancyGrid mapToSave;
	nav_msgs::Path trajectory;
	
	
public:
  	map_saver(){
  		sub_map = n.subscribe("/map", 1, &map_saver::map_callback, this); 
		sub_trajectory = n.subscribe("/trajectory", 1, &map_saver::trajectory_callback, this); 
		service = n.advertiseService("savemap", &map_saver::saveMap, this);

	}
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	  	ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		map.header.stamp.nsec = msg->header.stamp.nsec;
		map.header.stamp.sec = msg->header.stamp.sec;
		map.info = msg->info;
		map.data = msg->data; 
	}

	void trajectory_callback(const nav_msgs::Path::ConstPtr& msg){
	  	ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
		trajectory.header = msg->header;
		trajectory.poses = msg->poses; 
	}

	/**
	** resetPose
	Rosservice function callback called when a reqest from the resetPose service is received
	then a response is crafted.
	The structure of the service with reqest message and response message structures can be seen in project1/srv/ResertPose.srv
	@param req of type project1::ResetPose::Request&
	@param res of type project1::ResetPose::Response&
	*/
	bool saveMap(modifieddemo::SaveMap::Request &req, modifieddemo::SaveMap::Response &res) {
		ROS_INFO("%d, %d, %f",req.value,map.header.stamp.nsec,trajectory.poses[req.value].pose.position.x);
		res.result = 200;
		return true;
	}

};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "map_save_trajectory");
 	map_saver my_map_saver;
 	ros::spin();
 	return 0;
}
