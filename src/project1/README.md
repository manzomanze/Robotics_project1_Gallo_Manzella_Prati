# Project 1

## Goals
1. Compute odometry using appropriate kinematics:
    - Compute robot linear and angular velocities v, ⍵ from wheel encoders
    - Compute odometry using both Euler and Runge-Kutta integration (using *ROS parameter for initial pose*)
    - Calibrate (fine-tune) robot parameters to match ground truth
2. Compute wheel control speeds from v, ⍵
3. Add a service to reset the odometry to a specified pose (x,y,θ)
4. Use dynamic reconfigure to select between integration method

### Compute odometry
Write down the formula to compute v and ⍵, adapt the forumla to use the ticks in the bag, which are less noisy. Compute a rough estimate of v and ⍵ and publish them on topic **cmd_vel** of type *geometry_msgs/TwistStamped*. <br>
Now that we have the velocities, we need to compute the odometry. We use the Euler integration at first and then Runge-Kutta as well.

We add ROS parameters for the initial pose (x, y, ϑ).

---

## Here we have to look inside three bags in order to perform odometry of the robot

The only topic we are insterested in is: /wheel_states

THe only msg we are insterested in is: sensor_msgs/JointState

## The bags

We have three bags to analyze, in order to compute the odometry of the robot from which the bags are recorded. <br>
The data in these three bags show three different knimeatic movements of the robot, the first one is just linear, the second one only consider forward motions and in-place rotations while the third one is a freestyle. 

The parameters we are insterested in are:
- velocity: which is in Rad/min and is a noisy measure
- position: the number of thick of the encoder 
- time: just the timestamp at which the parameters are recorded

---

# Instructions

To make rosrun work i need to source the setup.bash in the workspace/devel folder after catkin_make has been executed

The **order** of the calls in CMakeLists.txt is important!

## useful commands

To recover **the structure of the message** sent about the encoder info: the message type is sensor_msgs/JointState from the topic /wheel_states
```
rosmsg info sensor_msgs/JointState
```

To recover the **list of topics**
```
rostopic list
```

To **run the node** that subscribes to the wheel encoders
```
rosrun project1 odometry
```

To get the **info of the topics** of the project that are sent through the messages from the bag
```
rosbag info path/to/bag
```

To get the **values of the messages** sent over the topic publisher
```
rostopic echo /publisher
```

When we want to let **Catkin compile the new node** added, meaning the file _.cpp_ in src, we must append to the file CMakeLists.txt
```
add_executable(subscriber src/sub.cpp) target_link_libraries(subscriber ${catkin_LIBRARIES})
```

To run the reconfigure rqt interface:
```
rosrun rqt_reconfigure rqt_reconfigure
```

To run the plot:
```
rqt_plot /robot/pose/pose/position/x:y:z
rqt_plot /odom/pose/pose/position/x:y:z
```

