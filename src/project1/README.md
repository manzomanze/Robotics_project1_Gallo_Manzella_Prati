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
Write down the formula to compute v and ⍵, adapt the forumla to use the ticks in the bag, which are less noisy. <br>
Compute a rough estimate of v and ⍵ and publish them on topic **cmd_vel** of type *geometry_msgs/TwistStamped*. <br>
Now that we have the velocities, we need to compute the odometry. We use the Euler integration at first and then Runge-Kutta as well.

We add ROS parameters for the initial pose (x, y, ϑ).

---

## Here we have to look inside three bags in order to perform odometry of the robot

The only topic we are insterested in is: /wheel_states

The only msg we are insterested in is: sensor_msgs/JointState

## The bags

We have three bags to analyze, in order to compute the odometry of the robot from which the bags are recorded. <br>
The data in these three bags show three different knimeatic movements of the robot, the first one is just linear, the second one only consider forward motions and in-place rotations while the third one is a freestyle. 

The parameters we are insterested in are:
- velocity: which is in Rad/min and is a noisy measure
- position: the number of thick of the encoder 
- time: just the timestamp at which the parameters are recorded

---

# Instructions

To make rosrun work to source the setup.bash in the workspace/devel folder after catkin_make has been executed
```
source workspace/devel/setup.bash
```
Then to run the entire project using the launchfile
```
roslaunch project1 pose_vel.launch
```
Once all the nodes have started as well as roscore four rqt_plot will show up, in some systems the scaling factor of the axis may be too great therefore  clicking on the home button on the rqt_plot may reset the scale and make the data visible.
A pair of rqt_plot are focused on the odometry information the other pair are focused on the wheel_rpm information.
The former two show, respectively the topic /odom x and y coordinate that are computed from the coded Kinematics and Integration as well as /robot that shows x and y from the recorded bag info.
The latter two show the rpm information of the wheels(***as in radians per minute***), respectively /wheel_states/velocity[i] shows the rpm that are coming from the bag; /wheel_rpm/rpm_fl etc show the rpm of the wheels as computed by the coded inverse kinematics.

The bag recorded messages can be played using
```
rosbag play ~/path/to/bag --pause -r 3
```
--pause is useful to let us choose when the bag starts and -r 3 sets the message rate to be 3 times faster

To reset the pose at any time we can use the service ResetPose substituting x, y and theta with the chosen values
```
rosservice call resetpose x y theta 
```

Dynamic Reconfigure can be used to set the Integration method at runtime
Using a cli command 0 stands for EULER and 1 for RUNGE-KUTTA
```
rosrun dynamic_reconfigure dynparam set /odometry integration_method 0
```
Using the gui utility 
```
rosrun rqt_reconfigure rqt_reconfigure
```
Rviz can be used to see in 3D the computed odom and ground truth as is presented in the useful command section

The **order** of the calls in CMakeLists.txt is important!

## useful commands

To recover **the structure of the message** sent regarding the encoder information: the message type is sensor_msgs/JointState from the topic /wheel_states
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

To reset the pose to position x,y and orientation theta substitute relevant values to x y and theta
```
rosservice call resetpose x y theta 
```

To run rviz and visualize the odometry
```
rviz
```
The provided config (either config.rviz or topdown.rviz) can be used to avoid the next settings and manage the arrow size as well as the setup of a topdown view
Manually click on **add** button in the lower left choose **odometry** from the list and set the **topic** to **/odom** after the nodes and roscore have been launched with the launchfile
The ground truth extracted from the bag messages can be used adding another odometry coming from **/gtpose** topic

To plot with plotjuggler first install
```
sudo apt install ros-melodic-plotjuggler-ros
```
then
```
rosrun plotjuggler plotjuggler
```
click on streaming section selelct the dropdown menu choose "ROS Topic Subscriber" 
select any interesting topic for example the computed odometry

run the bag a little to let it discover the values
then select and drag in the graph each value separately to plot x and y separately otherwise select multiple values and drag using the ***RIGHT BUTTON OF THE MOUSE*** on a graph to plot an xy plot