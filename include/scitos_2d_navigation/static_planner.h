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
#include <actionlib/client/terminal_state.h>
#include <perceive_tabletop_action/FindGoalPoseAction.h>
#include <move_base_msgs/MoveBaseAction.h>

// class header
class static_planner_node
{
public:
    
    // constructor, destructor
    static_planner_node();
    ~static_planner_node();
    
    // public check for occupied path
    void check_path(); // needs tons of exception handles!!!
    
protected:

    // callback for goal subscriber
    void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamicMapIn);
    
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
    bool init_map;
    double map_width;
    double map_height;
    double map_origin_x;
    double map_origin_y;
    double map_res;
    int mod_num;
    
    // flags
    bool debug;
    bool block_for_block;
    
    // listen to tf
    tf::TransformListener transform;
    
    // init
    costmap_2d::Costmap2DROS *costmap;
    navfn::NavfnROS navfn_plan;
    
    // make path available globally
    std::vector<geometry_msgs::PoseStamped> path;
    
    // counters
    int cb_counter;
    
    // clients
    actionlib::SimpleActionClient<perceive_tabletop_action::FindGoalPoseAction> *pose_client;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *move_base_client;
};


#endif // STATIC_PLANNER_H
