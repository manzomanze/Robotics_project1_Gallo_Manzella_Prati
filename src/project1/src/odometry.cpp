#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities

#define SEC_IN_MIN 60
#define N_WHEELS 4
#define N_WINDOWS 42
#define RESOLUTION (2*3.14/(4*N_WINDOWS))
#define GEAR_RATIO 5
#define X_WHEEL_DISTANCE 0.200
#define Y_WHEEL_DISTANCE 0.169
#define RADIUS 0.07
#define EVERY_N_MSG_TO_DENOISE 6

enum wheel_order {
  FL,
  FR,
  RL,
  RR
};

typedef struct {
  double vel[N_WHEELS];
  double count_ticks[N_WHEELS];
} t_wheel_pos_vel;

typedef struct {
  uint32_t seq;
  double time;

  t_wheel_pos_vel wheel_info;
} t_msg;


class OdometryCalculator{
public:

  t_msg createStruct(const sensor_msgs::JointState::ConstPtr& msg){
    t_msg actual_msg;

    actual_msg.seq = msg -> header.seq;
    
    actual_msg.time = msg -> header.stamp.toSec();
    
    for (int i = 0; i < N_WHEELS; i++)
    {
      actual_msg.wheel_info.vel[i] = msg -> velocity[i];
      actual_msg.wheel_info.count_ticks[i] = msg -> position[i];
      
    }
    
    return actual_msg;
  }

  geometry_msgs::TwistStamped publishMsg_cmd_vel(double linear_x, double linear_y, double angular_z){
    /* generate geometry_msgs::TwistStamped msg containing the linear velocity 
    on x and  y and the angular velocity around the z axis */
    geometry_msgs::TwistStamped cmd_vel_msg;
    
    
    cmd_vel_msg.header.frame_id = "robot_frame";
    cmd_vel_msg.header.stamp = ros::Time::now();
    
    cmd_vel_msg.twist.linear.x = linear_x;
    cmd_vel_msg.twist.linear.y = linear_y;
    cmd_vel_msg.twist.linear.z = 0.0;

    cmd_vel_msg.twist.angular.x = 0.0;
    cmd_vel_msg.twist.angular.y = 0.0;
    cmd_vel_msg.twist.angular.z = angular_z;


    // print count to screen
    // publish messages
    cmd_vel_publisher.publish(cmd_vel_msg);
    return cmd_vel_msg;
  }
  void calculateEulerIntegration(double linear_x, double linear_y, double angular_z, double samplingTime){
      robot_x += (linear_x* cos(robot_theta)+linear_y * sin(robot_theta))*samplingTime ;
      robot_y += (linear_x* sin(robot_theta)+linear_y * cos(robot_theta))*samplingTime;
      robot_theta += angular_z * samplingTime;
      ROS_INFO("Robot X [%f] Robot Y [%f] Robot Theta [%f]",robot_x,robot_y,robot_theta);
  }

  void calculateKinematics(double *computedVelEachNmsg ){
    robotLinearVelocityOnX = RADIUS/4*GEAR_RATIO*
              (computedVelEachNmsg[FL]+computedVelEachNmsg[FR]+computedVelEachNmsg[RL]+computedVelEachNmsg[RR]);

    robotLinearVelocityOnY = RADIUS/4*GEAR_RATIO*
              (computedVelEachNmsg[FL]-computedVelEachNmsg[FR]-computedVelEachNmsg[RL]+computedVelEachNmsg[RR]);

    robotAngularVelocity = RADIUS/4*GEAR_RATIO/(X_WHEEL_DISTANCE+Y_WHEEL_DISTANCE)*
              (-computedVelEachNmsg[FL]+computedVelEachNmsg[FR]-computedVelEachNmsg[RL]+computedVelEachNmsg[RR]);

    ROS_INFO("ROBOT LINEAR VELOCITY ON X %f",robotLinearVelocityOnX);
    ROS_INFO("ROBOT LINEAR VELOCITY ON Y %f",robotLinearVelocityOnY);
    ROS_INFO("ROBOT ANGULAR VELOCITY %f",robotAngularVelocity);
    publishMsg_cmd_vel(robotLinearVelocityOnX,robotLinearVelocityOnY,robotAngularVelocity);
    std::cout << std::endl;
  }

  //callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    actual_msg = createStruct(msg);
    
    // here we check if it is the first measure
    // if so, then we just save the actual message without computing anything
    // it's not a big deale because the first bunch of msgs are just in still position
    if(isFirstMeasure){
      prevMsgEachNmsg = actual_msg;
      isFirstMeasure = 0;
      ROS_INFO("This was the first message, no delta are computed");
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
        computedVelEachNmsg[i] = tickPerSec * RESOLUTION;
      }

      ROS_INFO("Message sequence: %d", actual_msg.seq);
      ROS_INFO("Time stamp: %f", actual_msg.time);
      ROS_INFO("Velocity (Rad/min): %g %g %g %g", actual_msg.wheel_info.vel[FL],
                                                actual_msg.wheel_info.vel[FR],
                                                actual_msg.wheel_info.vel[RL],
                                                actual_msg.wheel_info.vel[RR]);
    
      ROS_INFO("Tick count: %g %g %g %g", actual_msg.wheel_info.count_ticks[FL],
                                        actual_msg.wheel_info.count_ticks[FR],
                                        actual_msg.wheel_info.count_ticks[RL],
                                        actual_msg.wheel_info.count_ticks[RR]);

      ROS_INFO("Delta ticks each four MSGS: %f %f %f %f", deltaPosEachNmsg[FL], deltaPosEachNmsg[FR],
                                                          deltaPosEachNmsg[RL], deltaPosEachNmsg[RR]);

      ROS_INFO("Delta time each four MSGS: %f", deltaTime);

      ROS_INFO("Computed velocity each %d MSGS: %f %f %f %f", EVERY_N_MSG_TO_DENOISE, computedVelEachNmsg[FL], computedVelEachNmsg[FR],
                                                                computedVelEachNmsg[RL], computedVelEachNmsg[RR]);
      std::cout << std::endl;

      
      // we reset the message 
      prevMsgEachNmsg = actual_msg;

      calculateKinematics(computedVelEachNmsg);

      calculateEulerIntegration(robotLinearVelocityOnX,robotLinearVelocityOnY,robotAngularVelocity,deltaTime);

    }

  }

  //constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    robotLinearVelocityOnX = 0;
    robotLinearVelocityOnY = 0;
    robotAngularVelocity = 0;
    robot_x = 0.0;
    robot_y = 0.0;
    robot_theta = 0.0;
    cmd_vel_publisher = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);    
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::encoderCallback,this);
  }
  
private:
  int isFirstMeasure = 1;
  
  // here we use a variable to store the message only after 4 messages
  // the idea is to limit the noise
 
  t_msg actual_msg, prevMsgEachNmsg;

  double deltaTime;

  double robot_x;
  double robot_y;
  double robot_theta;

  // here we declare the variables used for the computation of the velocity with less noise (hopefully)
  int deltaMsgNumber = 0;
  double deltaPosEachNmsg[N_WHEELS];

  double robotLinearVelocityOnX;
  double robotLinearVelocityOnY;
  double robotAngularVelocity;
  
  ros::NodeHandle n;
  ros::Subscriber sub_encoder_wheel;
  ros::Publisher cmd_vel_publisher;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "speed_calculator");
  //odom object of class OdometryCalcualtor
  OdometryCalculator odom;

  ros::spin();

  return 0;
}
