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

    ROS_INFO("Velocity (Rad/min): %f %f %f %f", actual_msg.wheel_info.vel[FL],
                                                actual_msg.wheel_info.vel[FR],
                                                actual_msg.wheel_info.vel[RL],
                                                actual_msg.wheel_info.vel[RR]);
    
    ROS_INFO("Tick count: %f %f %f %f", actual_msg.wheel_info.count_ticks[FL],
                                        actual_msg.wheel_info.count_ticks[FR],
                                        actual_msg.wheel_info.count_ticks[RL],
                                        actual_msg.wheel_info.count_ticks[RR]);

    ROS_INFO(" ");
    return actual_msg;
  }

  //callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    t_msg actual_msg = createStruct(msg);
    
    // ROS_INFO("Sequence ID %d", actual_msg.seq);

    // int *vel = actual_msg.wheel_info.vel;
    // ROS_INFO("Velocity [%f],[%f],[%f],[%f]", vel[FRONT_LEFT], vel[FRONT_RIGHT],
    //                                           vel[REAR_LEFT], vel[REAR_RIGHT]);
    // // ROS_INFO("ODOMETRY velocity IF IT IS RadPM [%f],[%f],[%f],[%f]",msg->velocity[0]/60*42/(2*3.14),msg->velocity[1]/60*42/(2*3.14),
    // // msg->velocity[2]/60*42/(2*3.14),msg->velocity[3]/60*42/(2*3.14));
    // ROS_INFO("ODOMETRY position [%f],[%f],[%f],[%f]",msg->position[0],msg->position[1],msg->position[2],msg->position[3]);

    // ROS_INFO("TIME from msg %f",msg->header.stamp.toSec());
    // ROS_INFO("TIME previous msg %f", previousTime);    

    // deltaTime = msg->header.stamp.toSec()-previousTime;
    // previousTime = msg->header.stamp.toSec();
    // ROS_INFO("Delta time sec %f",deltaTime);
    
    // for(int arrayposition=0;arrayposition<4;arrayposition++){
    //   deltaPosition[arrayposition] = msg->position[arrayposition]-tempPosition[arrayposition];
    //   tempPosition[arrayposition] = msg->position[arrayposition]; 
    // }
    // ROS_INFO("DELTA position [%f],[%f],[%f],[%f]",deltaPosition[0],deltaPosition[1],deltaPosition[2],deltaPosition[3]);
    // ROS_INFO("ANGULAR WHEEL VELOCITY [%f],[%f],[%f],[%f]\n",deltaPosition[0]/deltaTime, deltaPosition[1]/deltaTime,
    //                                   deltaPosition[2]/deltaTime, deltaPosition[3]/deltaTime);
  }

  //constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    tempPosition[0] = 17269.000000;
    tempPosition[1] = 11412.000000;
    tempPosition[2] = 15462.000000;
    tempPosition[3] = 13165.000000;
    previousTime = ros::Time::now().toSec();
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::encoderCallback,this);
    //linear_anglular_velocities = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }
  
private:
  float tempPosition[4];
  float deltaPosition[4];
  double deltaTime,previousTime;

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
