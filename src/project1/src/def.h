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

