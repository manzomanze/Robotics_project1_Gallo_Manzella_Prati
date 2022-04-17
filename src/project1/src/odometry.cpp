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

  void calculateKinematics(double *computedVelEachFour ){
    robotLinearVelocityOnX = RADIUS/4*GEAR_RATIO*
              (computedVelEachFour[FL]+computedVelEachFour[FR]+computedVelEachFour[RL]+computedVelEachFour[RR]);

    robotLinearVelocityOnY = RADIUS/4*GEAR_RATIO*
              (computedVelEachFour[FL]-computedVelEachFour[FR]-computedVelEachFour[RL]+computedVelEachFour[RR]);

    robotAngularVelocity = RADIUS/4*GEAR_RATIO/(X_WHEEL_DISTANCE+Y_WHEEL_DISTANCE)*
              (-computedVelEachFour[FL]+computedVelEachFour[FR]-computedVelEachFour[RL]+computedVelEachFour[RR]);

    ROS_INFO("ROBOT LINEAR VELOCITY ON X %f",robotLinearVelocityOnX);
    ROS_INFO("ROBOT LINEAR VELOCITY ON Y %f",robotLinearVelocityOnY);
    ROS_INFO("ROBOT ANGULAR VELOCITY %f",robotAngularVelocity);
  }

  //callback called each time a message on topic /wheel_states is published by the bag
  void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) { 
    actual_msg = createStruct(msg);
    
    // here we check if it is the first measure
    // if so, then we just save the actual message without computing anything
    // it's not a big deale because the first bunch of msgs are just in still position
    if(isFirstMeasure){
      prevMsg = actual_msg;
      prevMsgEachFour = actual_msg;
      isFirstMeasure = 0;
      ROS_INFO("This was the first message, no delta are computed");
      return;
    }

    // delta time computed at each msg
    deltaTime = actual_msg.time - prevMsg.time;
    ROS_INFO("Delta time: %f", deltaTime);

    double computedVel[N_WHEELS];
    // this for cycle is used to store and compute the velocity starting form the ticks
    for (int i = 0; i < N_WHEELS; i++)
    {
      deltaPosition[i] = actual_msg.wheel_info.count_ticks[i] - prevMsg.wheel_info.count_ticks[i];
      double tickPerSec = (deltaPosition[i]/deltaTime);
      computedVel[i] = tickPerSec * RESOLUTION;
    }

    ROS_INFO("Delta ticks: %f %f %f %f", deltaPosition[FL], deltaPosition[FR],
                                          deltaPosition[RL], deltaPosition[RR]);

    ROS_INFO("Computed velocity: %f %f %f %f", computedVel[FL], computedVel[FR],
                                                computedVel[RL], computedVel[RR]);

    // print the velocity given in the bag, it is in Rad/min, so I divide by 60 to get the Rad/s
    ROS_INFO("Velocity from the bag (Rad/sec): %f %f %f %f", actual_msg.wheel_info.vel[FL]/SEC_IN_MIN/4,
                                                    actual_msg.wheel_info.vel[FR]/SEC_IN_MIN/4,
                                                    actual_msg.wheel_info.vel[RL]/SEC_IN_MIN/4,
                                                    actual_msg.wheel_info.vel[RR]/SEC_IN_MIN/4); 

    // here the previous message is updated
    prevMsg = actual_msg;

    // here I need to check that the four message are passed
    deltaMsgNumber = actual_msg.seq - prevMsgEachFour.seq;
    // we enter this if each 4 messages
    if(deltaMsgNumber == 4){
      // now the delta time is performed between msgs that are distant 4 msgs
      deltaTime = actual_msg.time - prevMsgEachFour.time;
      double computedVelEachFour[N_WHEELS];
      for (int i = 0; i < N_WHEELS; i++)
      {
        deltaPosEachFour[i] = actual_msg.wheel_info.count_ticks[i] - prevMsgEachFour.wheel_info.count_ticks[i]; 
        double tickPerSec = (deltaPosEachFour[i]/deltaTime);
        computedVelEachFour[i] = tickPerSec * RESOLUTION;
      }

      ROS_INFO("Delta ticks each four MSGS: %f %f %f %f", deltaPosEachFour[FL], deltaPosEachFour[FR],
                                                          deltaPosEachFour[RL], deltaPosEachFour[RR]);

      ROS_INFO("Delta time each four MSGS: %f", deltaTime);

      ROS_INFO("Computed velocity each four MSGS: %f %f %f %f", computedVelEachFour[FL], computedVelEachFour[FR],
                                                                computedVelEachFour[RL], computedVelEachFour[RR]);
      // we reset the message 
      prevMsgEachFour = actual_msg;

      calculateKinematics(computedVelEachFour);
    }


    std::cout << std::endl;

  }

  //constructor of the class OdometryCalculator
  OdometryCalculator(){ 
    robotLinearVelocityOnX = 0;
    robotLinearVelocityOnY = 0;
    robotAngularVelocity = 0;
    sub_encoder_wheel = n.subscribe("wheel_states", 1000, &OdometryCalculator::encoderCallback,this);
    //linear_anglular_velocities = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }
  
private:
  int isFirstMeasure = 1;
  
  t_msg prevMsg;
  // here we use a variable to store the message only after 4 messages
  // the idea is to limit the noise
  t_msg prevMsgEachFour;

  t_msg actual_msg;



  double deltaPosition[N_WHEELS];
  double deltaTime;

  // here we declare the variables used for the computation of the velocity with less noise (hopefully)
  int deltaMsgNumber = 0;
  double deltaPosEachFour[N_WHEELS];

  double robotLinearVelocityOnX;
  double robotLinearVelocityOnY;
  double robotAngularVelocity;
  
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
