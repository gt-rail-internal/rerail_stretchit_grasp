# rerail_stretchit_grasp
Contains ROS packages for generating grasp pose for stretch. 
# setup
1. Clone Repo to a your workspace
2. catkin_make
3. (optional: place the rosbag file to  "$(find grasp_pkg_tests)/bag_files" and current expected name is "perception_multiple_obj_sparse.bag." Can modify bag file details in "rerail_stretchit_grasp/grasp_pkg_tests/launch/grasp_rosbag.launch."
4. Setup a ros network with stretch  (if not using bag files and not runnning everything on the robot)
# Running on stretch
1. launch stretch driver (on robot)
2. launch stretch camera (on robot)
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
