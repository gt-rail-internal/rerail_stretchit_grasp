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

