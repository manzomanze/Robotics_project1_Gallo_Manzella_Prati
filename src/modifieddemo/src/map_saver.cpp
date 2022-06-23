#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "modifieddemo/SaveMap.h"

 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 //#include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <sensor_msgs/Image.h>


class map_saver {

private:
	ros::NodeHandle n; 
	ros::Subscriber sub_map;
	ros::Subscriber sub_trajectory;
	ros::ServiceServer service;
	nav_msgs::OccupancyGrid map;
	nav_msgs::OccupancyGrid mapToSave;
	nav_msgs::Path trajectory;
	sensor_msgs::Image image;
	cv_bridge::CvImagePtr cv_ptr;
	
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

		image.data = msg->data;
		image.header = msg->header;
		image.height = msg->info.height;
		image.width = msg->info.width;
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
		cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
		
		cv::line(image, 
			new Point(10, 200),        //p1
         	new Point(300, 200),       //p2
         	new Scalar(0, 0, 255),     //Scalar object for color
         	5                          //Thickness of the line
		)
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
