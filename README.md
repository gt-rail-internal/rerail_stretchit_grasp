# rerail_stretchit_grasp
Contains ROS packages for generating grasp pose for stretch. 
# setup
Incase of any issues refer to Notes below.
1. Clone Repo to a your workspace
2. clone rail_agile # will add this package to the repo
   ```
   cd rerail_stretchit_grasp
   git clone https://github.com/GT-RAIL/rail_agile.git
   ```
4. catkin_make
5. (optional: place the rosbag file to  "$(find grasp_pkg_tests)/bag_files" and current expected name is "perception_multiple_obj_sparse.bag." Can modify bag file details in "rerail_stretchit_grasp/grasp_pkg_tests/launch/grasp_rosbag.launch.")
6. Setup a ros network with stretch  (if not using bag files and not runnning everything on the robot)
7. Install the fetch_grasp_suggestin dependencies
   ```
   sudo apt-get install python-matplotlib
   pip install treeinterpreter
   ```
# Interfacing with the grasping module (For Task exector)
To run the required nodes launch:
   ```
   roslaunch stretch_fetch_grasp_bridge grasp_modular.launch
   ```

Use the "/stretch_grasp_pose_suggester" service. The service message type is as follows.  
```
sensor_msgs/PointCloud2 point_cloud # segmented for the object of interest
---
bool success
geometry_msgs/PoseStamped grasp_pose
```
The point cloud is the segmented point cloud of the object of interest, not the entire point cloud. 
For task executor: this should be the point cloud returned from the rerail_segmentation.
# Testing on stretch
1. launch stretch driver (on robot)
   ```
   roslaunch stretch_core stretch_driver.launch
   ```
2. launch stretch camera (on robot)
   ```
   roslaunch stretch_core d435i_high_resolution.launch
   ```
3. Launch segmentation nodes # Need to move this to rerail_segmentation
   ```
   roslaunch stretch_fetch_grasp_bridge segmentation_modular.launch 
   ```
4. Launch grasp nodes
   ```
   roslaunch stretch_fetch_grasp_bridge grasp_modular.launch
   ```
5. Testing with dummy task executor
   ```
   rosrun stretch_fetch_grasp_bridge task_executor_dummy.py
   ```
This execution will calculate the grasp pose of the first indexed object by rail segmentation and publish it to the topic called "/stretch_grasp/single_grasp_pose".


# Testing with ROS bag
For easy testing you could use a rosbag instead of runnning on the robot. The step to run using rosbag are as follows:
1. Launch the rosbag
   ```
   roslaunch grasp_pkg_tests grasp_rosbag.launch 
   ```
2. Launch segmentation nodes # Need to move this to rerail_segmentation
   ```
   roslaunch stretch_fetch_grasp_bridge segmentation_modular.launch 
   ```
3. Launch grasp nodes
   ```
   roslaunch stretch_fetch_grasp_bridge grasp_modular.launch
   ```
4. Testsing with dummy task executor
   ```
   rosrun stretch_fetch_grasp_bridge task_executor_dummy.py
   ```
# Notes
## plot.cpp causing build issues
while building rail agile. Need to add the following header to plot.cpp in   rail_agile/src/agile_grasp/plot.cpp:
```
#include <boost/thread/thread.hpp>
```
## rail segmentaion configuration
Rail segmentation relies on a config file that requires specifying the table region infront of the robot. Ensure that the parameters are set correctly.
## classifier node died while running grasp_suggestor.launch
In fetch_grasp_suggestion/scripts/classifier_node.py
change 
```
#!/usr/bin/env python
to
#!/usr/bin/env python3
```
## skleran not found
```
pip install -U scikit-learn==0.18.1
```

