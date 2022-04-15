#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //message type of encoder from bag
#include "geometry_msgs/TwistStamped.h" //message type for linear and angular velocities

#define SEC_IN_MIN 60
#define N_WHEELS 4

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
    ROS_INFO("Message sequence: %d", actual_msg.seq);

    actual_msg.time = msg -> header.stamp.toSec();
    ROS_INFO("Time stamp: %f", actual_msg.time);


    for (int i = 0; i < N_WHEELS; i++)
    {
      actual_msg.wheel_info.vel[i] = msg -> velocity[i];
      actual_msg.wheel_info.count_ticks[i] = msg -> position[i];
      
    }

    ROS_INFO("Velocity (Rad/min): %g %g %g %g", actual_msg.wheel_info.vel[FL],
                                                actual_msg.wheel_info.vel[FR],
                                                actual_msg.wheel_info.vel[RL],
                                                actual_msg.wheel_info.vel[RR]);
    
    ROS_INFO("Tick count: %g %g %g %g", actual_msg.wheel_info.count_ticks[FL],
                                        actual_msg.wheel_info.count_ticks[FR],
                                        actual_msg.wheel_info.count_ticks[RL],
                                        actual_msg.wheel_info.count_ticks[RR]);

    std::cout << std::endl;
    
    return actual_msg;
  }

  //callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    t_msg actual_msg = createStruct(msg);
    
    // here we check if it is the first measure
    // if so, then we just save the actual message without computing anything
    // it's not a big deale because the first bunch of msgs are just in still position
    if(isFirstMeasure){
      prevMsg = actual_msg;
      isFirstMeasure = 0;
      ROS_INFO("This was the first message, no delta are computed");
      return;
    }

    // this cycle has the puropose of computing the delta on the number of ticks
    for (int i = 0; i < N_WHEELS; i++)
    {
      deltaPosition[i] = actual_msg.wheel_info.count_ticks[i] - prevMsg.wheel_info.count_ticks[i];
    }

    ROS_INFO("Delta ticks: %f %f %f %f", deltaPosition[FL], deltaPosition[FR],
                                          deltaPosition[RL], deltaPosition[RR]);

    deltaTime = actual_msg.time - prevMsg.time;
    
    ROS_INFO("Delta time: %f", deltaTime);

    std::cout << std::endl;

    // here the previous message is updated
    prevMsg = actual_msg;

  }

  //constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::encoderCallback,this);
    //linear_anglular_velocities = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }
  
private:
  int isFirstMeasure = 1;
  
  t_msg prevMsg;

  double deltaPosition[4];
  double deltaTime;

  ros::NodeHandle n;
  ros::Subscriber sub_encoder_wheel;
  ros::Publisher linear_angular_velocities;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  //odom object of class OdometryCalcualtor
  OdometryCalculator odom;

  ros::spin();

  return 0;
}
