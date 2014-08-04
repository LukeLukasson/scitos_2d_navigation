#ifndef STATIC_PLANNER_H
#define STATIC_PLANNER_H

#include <ros/ros.h>

// datacentre for table perception (needs to be include in front of costmap_2d_ros and navn_ros for some reason...
#include "ros_datacentre/message_store.h"
#include <strands_perception_msgs/Table.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// actionlib stuff
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <perceive_tabletop_action/FindGoalPoseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <scitos_2d_navigation/UnleashStaticPlannerAction.h>

// play some sound
#include <sound_play/sound_play.h>

// class header
namespace scitos_2d_navigation
{
class StaticPlanner
{
public:
    
    // constructor, destructor
    StaticPlanner(std::string name);
    ~StaticPlanner();
        
    // server
	void execute_cb(const scitos_2d_navigation::UnleashStaticPlannerGoalConstPtr &goal);

protected:

    // callback for goal subscriber
    void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamicMapIn);
    
    // public check for occupied path
    void check_path(); // needs tons of exception handles!!!
    
    bool pose_is_reachable(const geometry_msgs::PoseStamped &check_pose);
    
    // ROS handles
    ros::NodeHandle nh;
    ros::Rate *loop_rate;
    
    // subscribers
    ros::Subscriber subGoal;
    ros::Subscriber subDynamicMap;
    
    // publisher
    costmap_2d::Costmap2DPublisher *pubTableMap;
    costmap_2d::Costmap2DPublisher *pubDynMap;
    
    // maps
    costmap_2d::Costmap2D *tableMap;
    costmap_2d::Costmap2D *dynMap;
    double map_width;
    double map_height;
    double map_origin_x;
    double map_origin_y;
    double map_res;
    int mod_num;
    
    // flags
    bool debug;
    bool block_for_block;
    bool got_map;
    bool got_goal;
    bool check_path_is_active;
    
    // listen to tf
    tf::TransformListener transform;
    
    // init
    costmap_2d::Costmap2DROS *costmap;
    costmap_2d::Costmap2DROS *global_costmap_copy;
    navfn::NavfnROS navfn_plan;
    navfn::NavfnROS navfn_check_global;
    
    // make path available globally
    std::vector<geometry_msgs::PoseStamped> path;
    
    // store original goal
    geometry_msgs::PoseStamped original_goal;
        
    // counters
    int cb_counter;
    
    // clients
    actionlib::SimpleActionClient<perceive_tabletop_action::FindGoalPoseAction> *pose_client;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_client;
    
    // sound client
    sound_play::SoundClient sound_client;
    
    // server
    actionlib::SimpleActionServer<scitos_2d_navigation::UnleashStaticPlannerAction> unleash_server;
    std::string action_name;

};
};

#endif // STATIC_PLANNER_H
