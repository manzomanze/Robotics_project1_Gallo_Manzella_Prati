http://wiki.ros.org/laser_assembler/Tutorials/HowToAssembleLaserScans

# Instructions
Running the first bag
to run the LaserMerger (that has already been set up with its static transforms)roslaunch project2 scan_merger.launch
`roslaunch project2 scan_merger.launch`
It publishes on the topic */scan_multi*
To see the merged values
`rostopic echo /scan_multi`

Rosbag may require to have the parameter `--clock` due to the difference in timestamp between the bag and the wall time
`rosbag play path/to/the/bag --pause --clock`

i tried to setup gmapping on launchfile but at the moment it does not work 
use this rosrun instead
`rosrun gmapping slam_gmapping scan:=/scan_multi _base_frame:=base_link _xmin:=-5 _xmax:=5 _ymin:=-5 _ymax:=5 _maxUrange:=10`
`rosrun gmapping slam_gmapping scan:=/scan_multi _odom_frame:=odom _base_frame:=base_link _xmin:=-2 _xmax:=2 _ymin:=-2 _ymax:=2 _maxUrange:=5`