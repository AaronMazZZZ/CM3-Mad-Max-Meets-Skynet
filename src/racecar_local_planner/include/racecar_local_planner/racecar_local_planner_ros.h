#ifndef RACECAR_LOCAL_PLANNER_ROS_H
#define RACECAR_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>

//message types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//transform
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

//costmap
#include <costmap_2d/costmap_2d_ros.h>

// dynamic reconfigure
#include <racecar_local_planner/RacecarLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

//std c++
#include <cmath>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


namespace racecar_local_planner
{

enum PidNum
{
  kp = 0,
  ki = 1,
  kd = 2
};

enum VelocityNum
{
  linear_vel = 0,
  angular_vel = 1
};

enum TimerNum
{
  curr = 0,
  last = 1,
  before_last = 2
};

/**
  * @class RacecarLocalPlannerROS
  * @brief Implements the actual abstract navigation stack routines of the racecar_local_planner plugin
  * @todo Escape behavior, more efficient obstacle handling
  */
class RacecarLocalPlannerROS : public nav_core::BaseLocalPlanner
{

public:
  /**
    * @brief Default constructor of the racecar plugin
    */
  RacecarLocalPlannerROS();

  /**
    * @brief  Destructor of the plugin
    */
  ~RacecarLocalPlannerROS();

  /**
    * @brief Initializes the racecar plugin
    * @param name The name of the instance
    * @param tf Pointer to a transform listener
    * @param costmap_ros Cost map representing occupied and free space
    */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
    * @brief Set the plan that the racecar local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
    * @brief  Check if the goal pose has been achieved
    * The actual check is performed in computeVelocityCommands(). 
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();


  bool pruneGlobalPlan(const geometry_msgs::Pose2D &robot_pose, std::vector<geometry_msgs::PoseStamped>& plan);


  bool getVelocityCommand(const std::vector<geometry_msgs::PoseStamped> &plan,
                         geometry_msgs::Twist &cmd_vel);

 /// \brief Transforms the global plan of the robot from the planner frame to the local frame (modified).
 /// \param tf: he plan to pass to the local planner
 /// \param global_plan: he plan to pass to the local planner
 /// \param global_frame: global frame
 /// \param transformed_plan: plan after transformed
 /// \param tf_plan_to_global: Transformation between the global plan and the global planning frame
 /// \return \c true if the global plan is transformed, \c false otherwise
 bool transformGlobalPlan(const tf::TransformListener &tf,
                          const std::vector<geometry_msgs::PoseStamped>& global_plan,
                          const std::string &global_frame,
                          std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                          tf::StampedTransform* tf_plan_to_global = NULL);

 double pointsDisSquare(double x0, const double y0, const double x1, const double y1);

 void OdomCallback(const nav_msgs::Odometry& msg);

 void reconfigureCB(RacecarLocalPlannerReconfigureConfig& config, uint32_t level);

 void resetController();
	
private:
   std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

   // flags
   bool initialized_; //!< Keeps track about the corfrect initialization of this class

   bool goal_reached_; //!< store whether the goal is reached or not

   // external objects (store weak pointers)
   costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
   costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
   tf::TransformListener* tf_; //!< pointer to Transform Listener

   std::string global_plan_frame_;//!< The frame in which the global plan
   std::string global_frame_;//!< The frame in which the controller will run
   std::string robot_base_frame_;//!< Used as the base frame id of the robot


   ros::Time last_time_; /// last time

   ros::Time curr_time_;///current time

   geometry_msgs::Pose2D robot_curr_pose_;/// robot current pose

   geometry_msgs::Pose2D robot_ref_pose_;/// robot reference pose

   geometry_msgs::Twist robot_ref_vel_;

   geometry_msgs::Twist robot_curr_vel_;/// Store current robot translational and angular velocity (vx, vy, omega)

   geometry_msgs::Twist last_out_vel_; /// last time output velocity

   geometry_msgs::Twist curr_out_vel_;

   geometry_msgs::Pose2D robot_front_mid_pose_;/// a pose in the front of robot mid

   double last_linear_error_;
   double last_angular_error_;
   double before_last_linear_error_;
   double before_last_angular_error_;

   double k_[2][3];/// pid argument for linear and angular velocity

   ros::Subscriber odom_sub_;

   ros::NodeHandle nh_;

   ros::Publisher pose_pub_;
   
   ros::Publisher path_pub_;

   ros::Publisher ackermann_pub_;

   dynamic_reconfigure::Server<racecar_local_planner::RacecarLocalPlannerReconfigureConfig> *dsrv_;

   double goal_tolerance_;
   double max_linear_velocity_;
   double max_angular_velocity_;


   //steering_angle pid
   double last_steering_angle_;
   double last_angle_error_;
   double before_last_angle_error_;
   double k_angle_[3];
   
   
};

}// end namespace racecar_local_planner



#endif // RACECAR_LOCAL_PLANNER_ROS_H_




