# rerail_stretchit_grasp
Contains ROS packages for generating grasp pose for stretch. 
# setup
1. Clone Repo to a your workspace
2. catkin_make
3. (optional: place the rosbag file to  "$(find grasp_pkg_tests)/bag_files" and current expected name is "perception_multiple_obj_sparse.bag." Can modify bag file details in "rerail_stretchit_grasp/grasp_pkg_tests/launch/grasp_rosbag.launch."
4. Setup a ros network with stretch  (if not using bag files and not runnning everything on the robot)
5. Install the fetch_grasp_suggestin dependencies
   ```
   sudo apt-get install python-matplotlib
   pip install treeinterpreter
   ```
6. Need to also pull the rail_agile package. (Will push it to github soon)
# Running on stretch
1. launch stretch driver (on robot)
2. launch stretch camera (on robot)
Alternative to 1 and 2 you can use a ROS bag file:
```
roslaunch grasp_pkg_tests grasp_rosbag.launch 
```
Ensure ROS bag file is present in the grasp_pkg_tests/bag_files
3.
```
roslaunch stretch_fetch_grasp_bridge grasp_suggestor.launch
```
4. rqt_service_caller
```
rosrun rqt_service_caller rqt_service_caller
```

The service name is: "stretch_grasp_pose_suggester"
The inputs to the service message structure is as follows:
```
int64 segment_no # the object no. which segemented blob to choose, pls note the order may change each time since segmentation is rerun
int64 rank_no # the gripper rank no. to return. As above each call resegments and resamples grasps. Thus results will not be consistent with each call.
---
bool success # if False probably no filtered grasp was found.
geometry_msgs/PoseStamped grasp_pose # the grasp pose # kindly note the pose could be inverted. That is upside down. Will fix this in the future. 
```
# Notes
## plot.cpp causing build issues
while building rail agile. Need to add the following header to plot.cpp in   rail_agile/src/agile_grasp/plot.cpp:
```
#include <boost/thread/thread.hpp>
```
## rail segmentaion configuration
Rail segmentation relies on a config file that requires specifying the table region infront of the robot. Ensure that the parameters are set correctly. 
