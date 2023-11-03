#include <fetch_grasp_suggestion/test_grasp_suggestion.h>

using std::string;

Tester::Tester() :
    pnh_("~"),
    execute_grasp_client_("/executor/execute_grasp")
{
  string segmentation_topic;
  pnh_.param<string>("segmentation_topic", segmentation_topic, "rail_segmentation/segmented_objects");
  pnh_.param<bool>("debug", debug_, true);

  if (debug_)
  {
    debug_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose_to_execute", 1);
    debug_publisher_array_ = pnh_.advertise<geometry_msgs::PoseArray>("pose_array_to_execute", 1);
    all_grasps_publisher_ = pnh_.advertise<geometry_msgs::PoseArray>("all_grasp_poses", 1);

  }

  test_subscriber_ = pnh_.subscribe("grasp_object", 1, &Tester::testCallback, this);
  test_agile_subscriber_ = pnh_.subscribe("grasp_object_agile", 1, &Tester::testAgileCallback, this);
  test_heuristic_subscriber_ = pnh_.subscribe("grasp_object_heuristic", 1, &Tester::testHeuristicCallback, this);
  test_heuristic_subscriber_all_objs_ = pnh_.subscribe("grasp_object_heuristic_all_objs", 1, &Tester::testHeuristicCallbackAllObjs, this);
  test_heuristic_subscriber_all_pose_ = pnh_.subscribe("grasp_object_heuristic_all_pose", 1, &Tester::testHeuristicCallbackAllPose, this);


  test_random_subscriber_ = pnh_.subscribe("grasp_object_random", 1, &Tester::testRandomCallback, this);
  objects_subscriber_ = n_.subscribe(segmentation_topic, 1, &Tester::objectsCallback, this);

  suggest_grasps_client_ = n_.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps");
  suggest_grasps_baseline_client_ =
      n_.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps_baseline");
  suggest_grasps_random_client_ =
      n_.serviceClient<fetch_grasp_suggestion::SuggestGrasps>("suggester/suggest_grasps_random");
  rank_grasps_client_ = n_.serviceClient<fetch_grasp_suggestion::PairwiseRank>("suggester/pairwise_rank");
}

void Tester::testCallback(const std_msgs::Int32 &msg)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = object_list_.objects[msg.data].point_cloud;
  if (!suggest_grasps_client_.call(suggest_grasps))
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return;
  }
  ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());

  // rank grasp suggestions
  fetch_grasp_suggestion::PairwiseRank pairwise_rank;
  if (!rank_grasps_client_.call(pairwise_rank))
  {
    ROS_INFO("Call to pairwise ranking failed!");
    return;
  }
  geometry_msgs::PoseArray poseList = pairwise_rank.response.grasp_list;
  ROS_INFO("Ranked %lu grasps.", poseList.poses.size());

  // execute best grasp
  fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  grasp_goal.index = msg.data;
  grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
  size_t i = 0;
  while(true)
  {
    ROS_INFO("Showing grasp %lu...", i);

    grasp_goal.grasp_pose.pose = poseList.poses[i];
    if (debug_)
    {
      debug_publisher_.publish(grasp_goal.grasp_pose);
    }

    ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
    string input;
    std::cin >> input;

    switch(input[0])
    {
      case ']':
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;

      case '[':
        if (i > 0)
        {
          i --;
        }
        break;

      case 'q':
        return;

      case 'e':
        execute_grasp_client_.sendGoal(grasp_goal);
        execute_grasp_client_.waitForResult(ros::Duration(60));
        fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

        if (execute_grasp_result->success)
        {
          ROS_INFO("Grasp execution complete!");
          return;
        }
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;
    }
  }
}

void Tester::testAgileCallback(const std_msgs::Int32 &msg)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = object_list_.objects[msg.data].point_cloud;
  if (!suggest_grasps_baseline_client_.call(suggest_grasps))
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return;
  }
  ROS_INFO("Received %lu grasps.", suggest_grasps.response.grasp_list.poses.size());

  geometry_msgs::PoseArray poseList = suggest_grasps.response.grasp_list;

  // execute best grasp
  fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  grasp_goal.index = msg.data;
  grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
  size_t i = 0;
  while(true)
  {
    ROS_INFO("Showing grasp %lu...", i);

    grasp_goal.grasp_pose.pose = poseList.poses[i];
    if (debug_)
    {
      debug_publisher_.publish(grasp_goal.grasp_pose);
    }

    ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
    string input;
    std::cin >> input;

    switch(input[0])
    {
      case ']':
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;

      case '[':
        if (i > 0)
        {
          i --;
        }
        break;

      case 'q':
        return;

      case 'e':
        execute_grasp_client_.sendGoal(grasp_goal);
        execute_grasp_client_.waitForResult(ros::Duration(60));
        fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

        if (execute_grasp_result->success)
        {
          ROS_INFO("Grasp execution complete!");
          return;
        }
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;
    }
  }
}

void Tester::testHeuristicCallback(const std_msgs::Int32 &msg)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = object_list_.objects[msg.data].point_cloud;
  if (!suggest_grasps_client_.call(suggest_grasps))
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return;
  }
  ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());

  geometry_msgs::PoseArray poseList = suggest_grasps.response.grasp_list;

  // execute best grasp
  fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  grasp_goal.index = msg.data;
  grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
  //for (size_t i = 0; i < poseList.poses.size(); i ++)
  size_t i = 0;
  while(true)
  {
    ROS_INFO("Showing grasp %lu...", i);

    grasp_goal.grasp_pose.pose = poseList.poses[i];
    if (debug_)
    {
      debug_publisher_.publish(grasp_goal.grasp_pose);
    }

    ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
    string input;
    std::cin >> input;

    switch(input[0])
    {
      case ']':
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;

      case '[':
        if (i > 0)
        {
          i --;
        }
        break;

      case 'q':
        return;

      case 'e':
        execute_grasp_client_.sendGoal(grasp_goal);
        execute_grasp_client_.waitForResult(ros::Duration(60));
        fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

        if (execute_grasp_result->success)
        {
          ROS_INFO("Grasp execution complete!");
          return;
        }
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;
    }
  }
}

void Tester::testHeuristicCallbackAllPose(const std_msgs::Int32 &msg)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);
  ROS_INFO("Inside testHeuristicCallbackStretch!!!");
  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = object_list_.objects[msg.data].point_cloud;
  if (!suggest_grasps_client_.call(suggest_grasps))
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return;
  }
  ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());

  geometry_msgs::PoseArray poseList = suggest_grasps.response.grasp_list;
  all_grasps_publisher_.publish(poseList);
  // execute best grasp
  // fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  // grasp_goal.index = msg.data;
  // grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
  // //for (size_t i = 0; i < poseList.poses.size(); i ++)
  // size_t i = 0;
}

void Tester::testHeuristicCallbackAllObjs(const std_msgs::Int32 &msg)
{
  // msg not used
  boost::mutex::scoped_lock lock(object_list_mutex_);
  geometry_msgs::PoseArray TopGraspList;

  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  for(int obj_no=0;obj_no<object_list_.objects.size();obj_no++)  
  {
    fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
    suggest_grasps.request.cloud = object_list_.objects[obj_no].point_cloud;
    if (!suggest_grasps_client_.call(suggest_grasps))
    {
      ROS_INFO("Call to suggest grasps service failed!");
      return;
    }
    if (suggest_grasps.response.grasp_list.poses.empty())
    {
      ROS_INFO("No grasp suggestions found, stopping execution.");
      return;
    }
    ROS_INFO("Received %lu grasp suggestions.", suggest_grasps.response.grasp_list.poses.size());

    geometry_msgs::PoseArray poseList = suggest_grasps.response.grasp_list;

    // execute best grasp
    fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
    grasp_goal.index = msg.data;
    grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
    TopGraspList.header.frame_id = poseList.header.frame_id;
    TopGraspList.poses.push_back(poseList.poses[0]);

    //for (size_t i = 0; i < poseList.poses.size(); i ++)
    size_t i = 0;
  }
  debug_publisher_array_.publish(TopGraspList);
  return;
  // while(true)
  // {
  //   ROS_INFO("Showing grasp %lu...", i);

  //   grasp_goal.grasp_pose.pose = poseList.poses[i];
  //   if (debug_)
  //   {
  //     // debug_publisher_.publish(grasp_goal.grasp_pose);
  //     debug_publisher_array_.publish(poseList);

  //   }

  //   ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
  //   string input;
  //   std::cin >> input;

  //   switch(input[0])
  //   {
  //     case ']':
  //       if (i < poseList.poses.size() - 1)
  //       {
  //         i ++;
  //       }
  //       break;

  //     case '[':
  //       if (i > 0)
  //       {
  //         i --;
  //       }
  //       break;

  //     case 'q':
  //       return;

  //     case 'e':
  //       execute_grasp_client_.sendGoal(grasp_goal);
  //       execute_grasp_client_.waitForResult(ros::Duration(60));
  //       fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

  //       if (execute_grasp_result->success)
  //       {
  //         ROS_INFO("Grasp execution complete!");
  //         return;
  //       }
  //       if (i < poseList.poses.size() - 1)
  //       {
  //         i ++;
  //       }
  //       break;
  //   }
  // }
}

void Tester::testRandomCallback(const std_msgs::Int32 &msg)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  if (msg.data >= object_list_.objects.size())
  {
    ROS_INFO("Object index out of bounds!");
    return;
  }

  // get grasp suggestions
  fetch_grasp_suggestion::SuggestGrasps suggest_grasps;
  suggest_grasps.request.cloud = object_list_.objects[msg.data].point_cloud;
  if (!suggest_grasps_random_client_.call(suggest_grasps))  //TODO: change to random grasps client
  {
    ROS_INFO("Call to suggest grasps service failed!");
    return;
  }
  if (suggest_grasps.response.grasp_list.poses.empty())
  {
    ROS_INFO("No grasp suggestions found, stopping execution.");
    return;
  }
  ROS_INFO("Received %lu grasps.", suggest_grasps.response.grasp_list.poses.size());

  geometry_msgs::PoseArray poseList = suggest_grasps.response.grasp_list;

  // execute best grasp
  fetch_grasp_suggestion::ExecuteGraspGoal grasp_goal;
  grasp_goal.index = msg.data;
  grasp_goal.grasp_pose.header.frame_id = poseList.header.frame_id;
  size_t i = 0;
  while(true)
  {
    ROS_INFO("Showing grasp %lu...", i);

    grasp_goal.grasp_pose.pose = poseList.poses[i];
    if (debug_)
    {
      debug_publisher_.publish(grasp_goal.grasp_pose);
    }

    ROS_INFO("Enter a command (e: execute; q: quit; [ or ]: back or forward): ");
    string input;
    std::cin >> input;

    switch(input[0])
    {
      case ']':
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;

      case '[':
        if (i > 0)
        {
          i --;
        }
        break;

      case 'q':
        return;

      case 'e':
        execute_grasp_client_.sendGoal(grasp_goal);
        execute_grasp_client_.waitForResult(ros::Duration(60));
        fetch_grasp_suggestion::ExecuteGraspResultConstPtr execute_grasp_result = execute_grasp_client_.getResult();

        if (execute_grasp_result->success)
        {
          ROS_INFO("Grasp execution complete!");
          return;
        }
        if (i < poseList.poses.size() - 1)
        {
          i ++;
        }
        break;
    }
  }
}

void Tester::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList &list)
{
  boost::mutex::scoped_lock lock(object_list_mutex_);

  object_list_ = list;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_grasp_suggestion");

  Tester t;

  ros::spin();

  return EXIT_SUCCESS;
}
