# Project 1

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
