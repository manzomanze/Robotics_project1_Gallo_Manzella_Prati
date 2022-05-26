# Instructions
Running the first bag
to run the LaserMerger (that has already been set up with its static transforms)roslaunch project2 scan_merger.launch
`roslaunch project2 scan_merger.launch`
It publishes on the topic */scan_multi*
To see the merged values
`rostopic echo /scan_multi`