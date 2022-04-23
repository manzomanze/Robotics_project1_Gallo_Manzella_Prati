#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "project1/ResetPose.h"

class Kinematics{
public:

  t_msg createStruct(const sensor_msgs::JointState::ConstPtr& msg){
    t_msg actual_msg;

    actual_msg.seq = msg -> header.seq;
    
    actual_msg.time = msg -> header.stamp.toSec();
    
    for (int i = 0; i < N_WHEELS; i++) {
    // ? remember the velocity is in rad/min and it's measured at the motor, not the wheel
    actual_msg.wheel_info.vel[i] = (msg -> velocity[i]) / (SEC_IN_MIN * GEAR_RATIO);
    actual_msg.wheel_info.count_ticks[i] = msg -> position[i];
      
    }
    
    return actual_msg;
  }

    //  Callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    actual_msg = createStruct(msg);
    
    // here we check if it is the first measure
    // if so, then we just save the actual message without computing anything
    // it's not a big deale because the first bunch of msgs are just in still position
    if(isFirstMeasure){
      prevMsgEachNmsg = actual_msg;
      isFirstMeasure = 0;
      if(DEBUG){
        ROS_INFO("This was the first message, no delta are computed");
      }
      
      
      

      // mat.getEulerYPR()       

      return;
    }

    // here I need to check that the four message are passed
    deltaMsgNumber = actual_msg.seq - prevMsgEachNmsg.seq;
    // we enter this if each N messages
    if(deltaMsgNumber == EVERY_N_MSG_TO_DENOISE){
      
      // now the delta time is performed between msgs that are distant 4 msgs
      deltaTime = actual_msg.time - prevMsgEachNmsg.time;
      double computedVelEachNmsg[N_WHEELS];
      for (int i = 0; i < N_WHEELS; i++)
      {
        deltaPosEachNmsg[i] = actual_msg.wheel_info.count_ticks[i] - prevMsgEachNmsg.wheel_info.count_ticks[i]; 
        double tickPerSec = (deltaPosEachNmsg[i]/deltaTime);
        computedVelEachNmsg[i] = tickPerSec * RESOLUTION / GEAR_RATIO; // ? should it be divided per gear ration? I think so
      }
      if(DEBUG){
        ROS_INFO("Message sequence: %d", actual_msg.seq);
        ROS_INFO("Time stamp: %f", actual_msg.time);
        ROS_INFO("Velocity (Rad/s): %g %g %g %g", actual_msg.wheel_info.vel[FL],
                                                  actual_msg.wheel_info.vel[FR],
                                                  actual_msg.wheel_info.vel[RL],
                                                  actual_msg.wheel_info.vel[RR]);
      
        ROS_INFO("Tick count: %g %g %g %g", actual_msg.wheel_info.count_ticks[FL],
                                          actual_msg.wheel_info.count_ticks[FR],
                                          actual_msg.wheel_info.count_ticks[RL],
                                          actual_msg.wheel_info.count_ticks[RR]);

        ROS_INFO("Delta ticks each %d MSGS: %f %f %f %f", EVERY_N_MSG_TO_DENOISE, deltaPosEachNmsg[FL], deltaPosEachNmsg[FR],
                                                        deltaPosEachNmsg[RL], deltaPosEachNmsg[RR]);
        ROS_INFO("Delta time each %d MSGS: %f", EVERY_N_MSG_TO_DENOISE, deltaTime);

        ROS_INFO("Computed velocity each %d MSGS: %f %f %f %f", EVERY_N_MSG_TO_DENOISE, computedVelEachNmsg[FL], computedVelEachNmsg[FR],
                                                                  computedVelEachNmsg[RL], computedVelEachNmsg[RR]);
        std::cout << std::endl;
      }

      // we reset the message 
      prevMsgEachNmsg = actual_msg;

      calculateKinematics(computedVelEachNmsg);


    }

  }

    /*
    Calculates the Direct Kinematics of the robot that is the
    speed of the robot (linear x and y and angular) 
    given the angular speed of each wheel

    It calculates the movement adding up to EVERY_N_MSG_TO_DENOISE messages
  */

    void calculateKinematics(double *computedVelEachNmsg ){
    robotLinearVelocityOnX = RADIUS_WHEEL/4*
            (computedVelEachNmsg[FL]+computedVelEachNmsg[FR]+computedVelEachNmsg[RL]+computedVelEachNmsg[RR]);

    robotLinearVelocityOnY = RADIUS_WHEEL/4*
            (-computedVelEachNmsg[FL]+computedVelEachNmsg[FR]+computedVelEachNmsg[RL]-computedVelEachNmsg[RR]);

    robotAngularVelocity = (RADIUS_WHEEL/4)/(X_WHEEL_DISTANCE+Y_WHEEL_DISTANCE)*
            (-computedVelEachNmsg[FL]+computedVelEachNmsg[FR]-computedVelEachNmsg[RL]+computedVelEachNmsg[RR]);
    if(DEBUG){
    ROS_INFO("ROBOT LINEAR VELOCITY ON X %f",robotLinearVelocityOnX);
    ROS_INFO("ROBOT LINEAR VELOCITY ON Y %f",robotLinearVelocityOnY);
    ROS_INFO("ROBOT ANGULAR VELOCITY %f",robotAngularVelocity);
    }
    publishMsg_cmd_vel(robotLinearVelocityOnX,robotLinearVelocityOnY,robotAngularVelocity);
    if(DEBUG){
    std::cout << std::endl;
    }
  }

    // Publishes robot velocities on the topic cmd_vel
  void publishMsg_cmd_vel(double linear_x, double linear_y, double angular_z){
    /* generate geometry_msgs::TwistStamped msg containing the linear velocity 
    on x and  y and the angular velocity around the z axis */
    geometry_msgs::TwistStamped cmd_vel_msg;
    
    
    cmd_vel_msg.header.frame_id = "robot_frame";
    cmd_vel_msg.header.stamp = ros::Time::now(); // ? non convien mettere il timestamp dell'ultimo messaggio ricevuto?
    
    cmd_vel_msg.twist.linear.x = linear_x;
    cmd_vel_msg.twist.linear.y = linear_y;
    cmd_vel_msg.twist.linear.z = 0.0;

    cmd_vel_msg.twist.angular.x = 0.0;
    cmd_vel_msg.twist.angular.y = 0.0;
    cmd_vel_msg.twist.angular.z = angular_z;


    // print count to screen
    // publish messages
    cmd_vel_publisher.publish(cmd_vel_msg);
    
  }

  Kinematics(){ 
    robotLinearVelocityOnX = 0;
    robotLinearVelocityOnY = 0;
    robotAngularVelocity = 0;
    float test = 0.0; 
    

    cmd_vel_publisher = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);    
    
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &Kinematics::encoderCallback,this);
  }

private:
    int isFirstMeasure = 1;

    // here we use a variable to store the message only after 4 messages
    // the idea is to limit the noise

    t_msg actual_msg, prevMsgEachNmsg;

    double deltaTime;

    double robotLinearVelocityOnX;
    double robotLinearVelocityOnY;
    double robotAngularVelocity;

    // here we declare the variables used for the computation of the velocity with less noise (hopefully)
    int deltaMsgNumber = 0;
    double deltaPosEachNmsg[N_WHEELS];

    ros::NodeHandle n;
    ros::Publisher cmd_vel_publisher;

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics");
  //odom object of class OdometryCalculator
  Kinematics robot_velocities;

  ros::spin();

  return 0;
}