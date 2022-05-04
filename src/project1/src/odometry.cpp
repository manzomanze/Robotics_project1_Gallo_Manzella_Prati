#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "project1/ResetPose.h"
#include <dynamic_reconfigure/server.h>
#include "project1/integMethodConfig.h" // include the dynamic reconfigure for the integration method
#include "param.h"
#include "def.h"
#include "cmath"

class OdometryCalculator{
public:
  /**
    **cmdVelCallback
    This callback is called each time a message on topic /cmd_vel from the kinematics class arrives
    It calls the proper function that implements the integration method depending on the value of 
    **currentIntgration 
    0 Euler
    1 RungeKutta
    It also calls publishMsg_odom to publish the odometry
    @param msg is a message of type 
    ** geometry_msgs::TwistStamped::ConstPtr&
  */
  void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    ROS_INFO("Received cmdVel %f %f %f",msg->twist.linear.x,msg->twist.linear.y,msg->twist.angular.z);
    if(currentIntegration!=0 && currentIntegration !=1){
      throw std::runtime_error("Error: Integration Method of dynamic reconfigure is out of range!!"); 
    }
    if(currentIntegration == 0){
      calculateEulerIntegration((double)msg->twist.linear.x,(double)msg->twist.linear.y,(double)msg->twist.angular.z,msg->header.stamp.toSec());

    }
    else{
      calculateRungeKuttaIntegration((double)msg->twist.linear.x,(double)msg->twist.linear.y,(double)msg->twist.angular.z,msg->header.stamp.toSec());
    }
      
    publishMsg_odom(robot_tf_odom_x,robot_tf_odom_y,robot_tf_odom_theta);
  }

  /**
    **param_callback
    This callback is called each time a new value on the dynamic_reconfigure configuration integration_method 
    It impacts the integration method used to compute the odometry by setting  
    **currentIntgration
    appropriately,
    0 Euler
    1 RungeKutta
    @param config is a configuration of dynamic_reconfigure of type 
    **  project1::integMethodConfig&
    @param level of type uint32_t
  */
  void param_callback( project1::integMethodConfig &config, uint32_t level){
    
    ROS_INFO("Reconfigure request, new values are: %d", config.integration_method);
    currentIntegration = config.integration_method;
    ROS_INFO("CURRENT INTEGRATION %d",currentIntegration);

}
  
   /** 
   **publishMsg_odom
   Publishes robot odometry on the topic /odom 
   @param publish_odom_x x position in meters of the robot with respect to the odom tf  
   @param publish_odom_y y position in meters of the robot with respect to the odom tf 
   @param publish_odom_theta theta orientation in radians of the robot with resepect to the odom tf
   */
  void publishMsg_odom(double publish_odom_x, double publish_odom_y, double publish_odom_theta){
    /* generate nav_msgs::Odometry msg containing the position 
    on x and  y and the angular position around the z axis */
    nav_msgs::Odometry odom_msg;
    
    
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp = ros::Time::now();
    
    odom_msg.pose.pose.position.x = publish_odom_x;
    odom_msg.pose.pose.position.y = publish_odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    
    // Create this quaternion from roll/pitch/yaw (in radians)
    quat_tf.setRPY( 0, 0,  publish_odom_theta); //to transform in radians if not in radians
    tf2::convert(quat_tf,quat_msg);
    odom_msg.pose.pose.orientation = quat_msg;
    // print count to screen

    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    ROS_INFO("sent to odom %f %f",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y);
    // publish message
    odom_publisher.publish(odom_msg);
  }

  /**
  **calculateEulerIntegration
  Calculates the odometry using Euler Integration Method
  Computes the composite velocity wrt the robot internal frame of reference projected on x then on y
  multiplying for the delta time called
  **samplingTime
  The x position and y position are modified
  The orientation robot_tf_odom_theta is modified accounting for the samplingTime and the angularvelocity angular_z
  @param linear_velocity_x 
  @param linear_velocity_y
  @param angular_velocity_z 
  */
  void calculateEulerIntegration(double linear_velocity_x, double linear_velocity_y, double angular_velocity_z, double samplingTime){
    double vel_kx = (linear_velocity_x* cos(robot_tf_odom_theta) - linear_velocity_y * sin(robot_tf_odom_theta));
    robot_tf_odom_x += vel_kx *samplingTime;

    double vel_ky = (linear_velocity_x* sin(robot_tf_odom_theta) + linear_velocity_y * cos(robot_tf_odom_theta));
    robot_tf_odom_y += vel_ky*samplingTime;
    
    robot_tf_odom_theta += angular_velocity_z * samplingTime;
  
    if(1){
      ROS_INFO("EULER");
      ROS_INFO("Robot X [%f] Robot Y [%f] Robot Theta [%f]",robot_tf_odom_x,robot_tf_odom_y,robot_tf_odom_theta);
    }
  }
  /**
  **calculateRungeKuttaIntegration
  Calculates the odometry using Runge-Kutta Integration Method
  The rungeKuttaAdditionalRotation is computed to better integrate
  Computes the composite velocity wrt the robot internal frame of reference projected on x then on y
  multiplying for the delta time called
  **samplingTime
  The x position and y position are modified
  The orientation robot_tf_odom_theta is modified accounting for the samplingTime and the angularvelocity angular_z
  @param linear_velocity_x 
  @param linear_velocity_y
  @param angular_velocity_z 
  */
  void calculateRungeKuttaIntegration(double linear_velocity_x, double linear_velocity_y, double angular_velocity_z, double samplingTime){
    double rungeKuttaAdditionalRotation = angular_velocity_z*samplingTime/2;
    double vel_kx = (linear_velocity_x* cos(robot_tf_odom_theta + rungeKuttaAdditionalRotation)-linear_velocity_y * sin(robot_tf_odom_theta + rungeKuttaAdditionalRotation));
    robot_tf_odom_x += vel_kx *samplingTime ;

    double vel_ky = (linear_velocity_x* sin(robot_tf_odom_theta + rungeKuttaAdditionalRotation)+linear_velocity_y * cos(robot_tf_odom_theta + rungeKuttaAdditionalRotation));
    robot_tf_odom_y += vel_ky*samplingTime;

    ROS_INFO("velocities computed to reference axis %f %f",vel_kx,vel_ky);
    robot_tf_odom_theta += angular_velocity_z * samplingTime;
    if(1){
      ROS_INFO("RKKKKK");
      ROS_INFO("Robot X [%f] Robot Y [%f] Robot Theta [%f]",robot_tf_odom_x,robot_tf_odom_y,robot_tf_odom_theta);
    }
  }


 

  /**
  ** resetPose
  Rosservice function callback called when a reqest from the resetPose service is received
  then a response is crafted.
  The structure of the service with reqest message and response message structures can be seen in project1/srv/ResertPose.srv
  @param req of type project1::ResetPose::Request&
  @param res of type project1::ResetPose::Response&
  */
  bool resetPose(project1::ResetPose::Request &req, project1::ResetPose::Response &res) {
    robot_tf_odom_x = req.linearx;
    robot_tf_odom_y = req.lineary;
    robot_tf_odom_theta = req.angulartheta;
    ROS_INFO("Set to:%f, %f, %f", robot_tf_odom_x, robot_tf_odom_y, robot_tf_odom_theta);

    res.result = 200;
    return true;
  }

  // Constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    robot_tf_odom_x = 0.0;
    robot_tf_odom_y = 0.0;
    robot_tf_odom_theta = 0.0;
    currentIntegration = 0;
    f = boost::bind(&OdometryCalculator::param_callback,this, _1, _2); 
    dynServ.setCallback(f);

    service = n.advertiseService("resetpose", &OdometryCalculator::resetPose, this);
  
    cmd_vel_subscribe = n.subscribe("cmd_vel", 1000, &OdometryCalculator::cmdVelCallback,this);    
    odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1000); 
  }
  
private:
  
  int currentIntegration;

  // robot pose (Position and Orientation) in a fixed Frame of Reference
  double robot_tf_odom_x;
  double robot_tf_odom_y;
  double robot_tf_odom_theta;
  
  ros::NodeHandle n;
  ros::Subscriber cmd_vel_subscribe;
  ros::Publisher odom_publisher;
  ros::ServiceServer service;
  //dynamic_reconfigure config for the integrationMethod is set
  // in the /project1/cfg/integMethod.cfg 
  dynamic_reconfigure::Server<project1::integMethodConfig> dynServ;
  dynamic_reconfigure::Server<project1::integMethodConfig>::CallbackType f;

  
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  //odom object of class OdometryCalculator

 


  OdometryCalculator odom;

  ros::spin();

  return 0;
}
