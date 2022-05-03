#include "ros/ros.h"
#include "param.h"
#include "project1/parametersConfig.h"
#include "geometry_msgs/PoseStamped.h"




//odometry
typedef struct t_wheel_pos_vel{
  double vel[N_WHEELS];
  double count_ticks[N_WHEELS];
} t_wheel_pos_vel;

typedef struct t_msg {
  uint32_t seq;
  double time;
  t_wheel_pos_vel wheel_info;
} t_msg;

// class OdometryCalculator{
//   public:
//     t_msg createStruct(const sensor_msgs::JointState::ConstPtr& msg);
//     void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg);
//     void publishMsg_cmd_vel(double linear_x, double linear_y, double angular_z);
//     void publishMsg_odom(double robot_x, double robot_y, double robot_theta);
//     void calculateEulerIntegration(double linear_x, double linear_y, double angular_z, double samplingTime);
//     void calculateRungeKuttaIntegration(double linear_x, double linear_y, double angular_z, double samplingTime);
//     void calculateKinematics(double *computedVelEachNmsg );
//     bool resetPose(project1::ResetPose::Request &req, project1::ResetPose::Response &res);
    
// };


//gt_pose
struct position_t;
struct orientation_t;
struct pose_t;

enum wheel_order {
  FL,
  FR,
  RL,
  RR
};

enum integration_method_enum {
  EULER,
  RK
};

void printPosition(position_t position);
void printOrientation(orientation_t orient);
pose_t createStruct(const geometry_msgs::PoseStamped::ConstPtr& msg);

// class PoseClass { 
//   void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
//   void param_callback(double* xPos, double* yPos, double* zPos,
//                     double* xOrient, double* yOrient, double* zOrient, double* wOrient,
//                     project1::parametersConfig &config, uint32_t level);

// };