#include <racecar_local_planner/racecar_local_planner_ros.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(racecar_local_planner::RacecarLocalPlannerROS, nav_core::BaseLocalPlanner)



namespace racecar_local_planner
{
  
RacecarLocalPlannerROS::RacecarLocalPlannerROS()
  :initialized_(false),
    goal_reached_(false),
    last_linear_error_(0),
    last_angular_error_(0),
    before_last_linear_error_(0),
    before_last_angular_error_(0),
    costmap_ros_(NULL),
    costmap_(NULL),
    tf_(NULL),
    nh_("~/racecar_local_planner"),
    before_last_angle_error_(0),
    last_angle_error_(0)
{


//  k_[kp] = 0.9;
//  k_[ki] = 1.905;
//  k_[kd] = 0.0;
//  k_[kp] = 0.6;
//  k_[ki] = 1.5;
//  k_[kd] = 0.0;

  k_angle_[kp] = 0.9;
  k_angle_[ki] = 0.0;
  k_angle_[kd] = 0.0;


  goal_tolerance_ = 0.5;
  max_linear_velocity_ = 0.3;
  max_angular_velocity_ = 0.6;
}


RacecarLocalPlannerROS::~RacecarLocalPlannerROS()
{
}


void RacecarLocalPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(initialized_)
  {
    ROS_WARN("racecar_local_planner has already been initialized, doing nothing.");
    return;
  }else{

    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

    global_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    initialized_ = true;

    last_time_ = curr_time_ = ros::Time::now();

    odom_sub_ = nh_.subscribe("/vesc/odom", 1, &RacecarLocalPlannerROS::OdomCallback, this);

    // setup dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<racecar_local_planner::RacecarLocalPlannerReconfigureConfig>(ros::NodeHandle("~/" + name));
    dynamic_reconfigure::Server<racecar_local_planner::RacecarLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(
            &RacecarLocalPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ref_pose", 1);
    
    path_pub_ = nh_.advertise<nav_msgs::Path>("prun_path", 1);

    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0", 1);

    ROS_DEBUG("racecar_local_planner plugin initialized.");
  }
}

bool RacecarLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if(!initialized_)
  {
    ROS_ERROR("racecar_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("New Global Plan");
  resetController();

  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset goal_reached_ flag
  goal_reached_ = false;
  

  return true;
}


bool RacecarLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{

  if(!initialized_)
  {
    ROS_ERROR("racecar_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  //default cmd_vel
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  goal_reached_ = false;

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  // cause the transfomation between global plan and global frame will changed when robot move,
  // so we call the function in computeVelocityCommands() function,other in setPlan() function
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  tf::StampedTransform tf_plan_to_global;
  if(!transformGlobalPlan(*tf_,
                          global_plan_,
                          global_frame_,
                          transformed_plan,
                          &tf_plan_to_global))
  {
    ROS_WARN("transform global plan error");
    return false;
  }


  // prune transformed plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(robot_curr_pose_, transformed_plan);
  
  //for view
  std::size_t gsize = transformed_plan.size();
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "/odom";
  path.poses.resize(gsize);
  for(int i = 0; i < gsize; i++)
  {
	 path.poses[i] = transformed_plan[i];
  }
  path_pub_.publish(path);


  //check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  global_goal = transformed_plan.back();
  double dis_squar = pointsDisSquare(global_goal.pose.position.x,
                                     global_goal.pose.position.y,
                                     robot_curr_pose_.x,
                                     robot_curr_pose_.y);
  double dis = fabs(std::sqrt(dis_squar));
  std::cout << "dis of robot to goal:" << dis << "\n";
  if(dis < goal_tolerance_)
  {
    goal_reached_ = true;
    return true;
  }

  // Get the velocity command for this sampling interval
  if (!getVelocityCommand(transformed_plan,
                          cmd_vel))
  {
    ROS_WARN("getVelocityCommand error");
    return false;
  }

  return true;
}



bool RacecarLocalPlannerROS::isGoalReached()
{
  if(goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

bool RacecarLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::Pose2D &robot_pose,
                                             std::vector<geometry_msgs::PoseStamped> &plan)
{
  if(plan.empty())
    return true;

  /// get distance of robot to the last goal distance
  std::vector<geometry_msgs::PoseStamped>::iterator it_end = plan.end()-1;
  double dist_thresh_sq = pointsDisSquare(robot_pose.x,
                                          robot_pose.y,
                                          it_end->pose.position.x,
                                          it_end->pose.position.y);

  // iterate plan until a pose close to the last goal than robot
  std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
  std::size_t erase_end = 0;
  while ((it + erase_end) != plan.end())
  {
    double dist_sq = pointsDisSquare(it_end->pose.position.x,
                                     it_end->pose.position.y,
                                     (it + erase_end)->pose.position.x,
                                     (it + erase_end)->pose.position.y);
    if (dist_sq + 0.3 < dist_thresh_sq)
    {
       break;
    }
    erase_end++;
  }

  //erase both plan and global_plan_
  if((it + erase_end) != plan.end())
  {
    plan.erase(plan.begin(), (it + erase_end));
    global_plan_.erase(global_plan_.begin(), (global_plan_.begin() + erase_end));
  }

  return true;

}

bool RacecarLocalPlannerROS::getVelocityCommand(const std::vector<geometry_msgs::PoseStamped>&plan,
                                                geometry_msgs::Twist& cmd_vel)
{
  if(plan.empty())
  {
    ROS_INFO("The local plan is empty");
    return false;
  }

  curr_time_ = ros::Time::now();

  //get next ref pose
  geometry_msgs::PoseStamped goal;
  goal = plan.back();
  double dis_robot_to_goal = pointsDisSquare(goal.pose.position.x,
                                             goal.pose.position.y,
                                             robot_curr_pose_.x,
                                             robot_curr_pose_.y);
  //until the pose closer to the last goal than robot or the end pose found
  geometry_msgs::PoseStamped pose;
  for(int i = 0; i < plan.size(); i++)
  {
    //dis to the last goal
    pose = plan[i];
    double dis = pointsDisSquare(pose.pose.position.x,
                                 pose.pose.position.y,
                                 robot_curr_pose_.x,
                                 robot_curr_pose_.y);
    if((dis < dis_robot_to_goal) || (i == plan.size() - 1))
    {
      break;
    }
  }
  robot_ref_pose_.x = pose.pose.position.x;
  robot_ref_pose_.y = pose.pose.position.y;
  robot_ref_pose_.theta = atan2(robot_ref_pose_.y - robot_curr_pose_.y, robot_ref_pose_.x - robot_curr_pose_.x);

  //get delta time
  double delta_time = (curr_time_ - last_time_).toSec();

  //compute errors
  double theta = atan2(sin(robot_curr_pose_.theta), cos(robot_curr_pose_.theta));
  double curr_angle_error = robot_ref_pose_.theta - theta;

  //compute pid factors
  double a, b, c;
  a = k_angle_[kp] + k_angle_[ki]*delta_time/2.0 + k_angle_[kd]*delta_time;
  b = k_angle_[kp] + k_angle_[ki]*delta_time/2.0 - 2*k_angle_[kd]*delta_time;
  c = k_angle_[kd]/delta_time;

  //compute the volocity
  double curr_steering_angle = last_steering_angle_
      + a * curr_angle_error
      + b * last_angle_error_
      + c * before_last_angle_error_;

  if(curr_angle_error > 0)
    curr_steering_angle = 0.3;
  else
    curr_steering_angle = -0.3;

  std::cout << "curr_steering_angle: " << curr_steering_angle
            << "curr_angle_error" << curr_angle_error
            << std::endl;

  double L = 0.2;
  curr_out_vel_.linear.x = 0.5;
  curr_out_vel_.linear.y = 0;

  //limit reference velocity
  if(curr_out_vel_.linear.x > max_linear_velocity_)
    curr_out_vel_.linear.x = max_linear_velocity_;
  else if(curr_out_vel_.linear.x < (-1)*max_linear_velocity_)
    curr_out_vel_.linear.x = (-1)*max_linear_velocity_;

  if(curr_out_vel_.angular.z > max_angular_velocity_)
    curr_out_vel_.angular.z = max_angular_velocity_;
  else if(curr_out_vel_.angular.z < (-1)*max_angular_velocity_)
    curr_out_vel_.angular.z = (-1)*max_angular_velocity_;

  //pub ackermann speed
  ackermann_msgs::AckermannDriveStamped msg;
  msg.header.frame_id = "/base_link";
  msg.header.stamp = ros::Time::now();
  msg.drive.jerk = 1.0;
  msg.drive.acceleration = 1;
  msg.drive.speed = curr_out_vel_.linear.x;
  msg.drive.steering_angle = curr_steering_angle;
  msg.drive.steering_angle_velocity = 0;
  ackermann_pub_.publish(msg);


  //for next iteration
  last_time_ = curr_time_;

  last_angle_error_ = curr_angle_error;
  before_last_angle_error_ = last_angle_error_;
  last_steering_angle_ = curr_steering_angle;

  //for view
  geometry_msgs::PoseStamped ref_pose;
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.header.frame_id = global_frame_;
  ref_pose.pose.position.x = robot_ref_pose_.x;
  ref_pose.pose.position.y = robot_ref_pose_.y;
  ref_pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_ref_pose_.theta);
  pose_pub_.publish(ref_pose);



  return true;
}

bool RacecarLocalPlannerROS::transformGlobalPlan(const tf::TransformListener& tf,
                                                 const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                                 const std::string& global_frame,
                                                 std::vector<geometry_msgs::PoseStamped> &transformed_plan,
                                                 tf::StampedTransform* tf_plan_to_global)
{
  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try
  {
    //get the plan_to_global_transform plan frame to global_frame
    tf::StampedTransform plan_to_global_transform;
    tf.waitForTransform(global_frame, ros::Time::now(),
                        plan_pose.header.frame_id, plan_pose.header.stamp,
                        plan_pose.header.frame_id, ros::Duration(3.0));
    tf.lookupTransform(global_frame, ros::Time(),
                       plan_pose.header.frame_id, plan_pose.header.stamp,
                       plan_pose.header.frame_id, plan_to_global_transform);

    //transform all global plan pose
    geometry_msgs::PoseStamped newer_pose;
    tf::Stamped<tf::Pose> tf_pose;
    for(int i = 0 ; i < global_plan.size(); i++)
    {
      const geometry_msgs::PoseStamped pose = global_plan[i];
      //pose to tf pose
      tf::poseStampedMsgToTF(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf::poseStampedTFToMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);
    }

    if(tf_plan_to_global)
      *tf_plan_to_global = plan_to_global_transform;


  }catch(tf::LookupException& ex)
  {
    ROS_WARN("LookupException error");
    return false;
  }catch(tf::ExtrapolationException& ex)
  {
    ROS_WARN("ExtrapolationException error");
    return false;
  }catch(tf::ConnectivityException& ex)
  {
    ROS_WARN("ConnectivityException error");
    return false;
  }

  return true;

}

double RacecarLocalPlannerROS::pointsDisSquare(const double x0,
                                               const double y0,
                                               const double x1,
                                               const double y1)
{
 double dis = 0;
 dis = (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1);
 return dis;
}

void RacecarLocalPlannerROS::OdomCallback(const nav_msgs::Odometry &msg)
{
  //get robot pose
  robot_curr_pose_.x = msg.pose.pose.position.x;
  robot_curr_pose_.y = msg.pose.pose.position.y;
  robot_curr_pose_.theta = tf::getYaw(msg.pose.pose.orientation);

  //trans a point from /base_link to /odom
  tf::StampedTransform base_to_odom_transform;
  ros::Time now = ros::Time::now();

  tf_->waitForTransform("/odom",
                      "/base_link",
                      now,
                      ros::Duration(3.0));
  tf_->lookupTransform("/odom",
                     "/base_link",
                     now,
                     base_to_odom_transform);

  geometry_msgs::PoseStamped base_pose;
  geometry_msgs::PoseStamped odom_pose;
  tf::Stamped<tf::Pose> tf_pose;

  base_pose.header.frame_id = "/base_link";
  base_pose.header.stamp = ros::Time::now();
  base_pose.pose.position.x = 0.6;
  base_pose.pose.position.y = 0.0;
  base_pose.pose.position.z = 0.0;
  tf::poseStampedMsgToTF(base_pose, tf_pose);
  tf_pose.setData(base_to_odom_transform * tf_pose);
  tf_pose.stamp_ = base_to_odom_transform.stamp_;
  tf_pose.frame_id_ = "/odom";
  tf::poseStampedTFToMsg(tf_pose, odom_pose);

  geometry_msgs::PoseStamped ref_pose;
  ref_pose.header.stamp = ros::Time::now();
  ref_pose.header.frame_id = "/odom";
  ref_pose.pose.position.x = odom_pose.pose.position.x;
  ref_pose.pose.position.y = odom_pose.pose.position.y;
  pose_pub_.publish(ref_pose);

  robot_curr_pose_.x = odom_pose.pose.position.x;
  robot_curr_pose_.x = odom_pose.pose.position.x;

  //Get robot velocity
  robot_curr_vel_.linear.x = msg.twist.twist.linear.x;
  robot_curr_vel_.linear.y = msg.twist.twist.linear.y;
  robot_curr_vel_.angular.z = msg.twist.twist.angular.z;
}

void RacecarLocalPlannerROS::reconfigureCB(RacecarLocalPlannerReconfigureConfig& config, uint32_t level)
{
//  k_[kp] = config.linear_kp;
//  k_[ki] = config.linear_ki;
//  k_[kd] = config.linear_kd;
//  k_[kp] = config.angular_kp;
//  k_[ki] = config.angular_ki;
//  k_[kd] = config.angular_kd;
  max_angular_velocity_ = config.max_angular_velocity;
  max_linear_velocity_ = config.max_linear_velocity;
  goal_tolerance_ = config.goal_tolerance;
  ROS_INFO("linear: kp:%f, ki:%f, kd:%f\n"
           "angular: kp:%f, ki:%f, kd:%f\n"
           "max angular: %f\n"
           "max linear: %f\n"
           "goal tolerance: %f",
           config.linear_kp,
           config.linear_ki,
           config.linear_kd,
           config.angular_kp,
           config.angular_ki,
           config.angular_kd,
           config.max_angular_velocity,
           config.max_linear_velocity,
           config.goal_tolerance);
}

void RacecarLocalPlannerROS::resetController()
{
  goal_reached_ = false;
  last_linear_error_ = 0;
  last_angular_error_ = 0;
  before_last_linear_error_ = 0;
  before_last_angular_error_ = 0;

  robot_ref_pose_ = robot_curr_pose_;/// robot reference pose

  robot_ref_vel_.linear.x = 0;
  robot_ref_vel_.linear.y = 0;
  robot_ref_vel_.angular.z = 0;

  robot_curr_vel_ = robot_ref_vel_;
  last_out_vel_ = robot_ref_vel_;
  curr_out_vel_ = robot_ref_vel_;

  last_time_ = curr_time_;

  last_angle_error_ = 0;
  before_last_angle_error_ = 0;
}




}/// namespace racecar_local_planner


































