#ifndef UNLEASH_STATIC_PLANNER_H
#define UNLEASH_STATIC_PLANNER_H

#include <ros/ros.h>

// actionlib stuff
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <scitos_2d_navigation/UnleashStaticPlannerAction.h>

// recovery behavior
#include <nav_core/recovery_behavior.h>

// class header
namespace scitos_2d_navigation
{
class UnleashStaticPlannerRecovery : public nav_core:: RecoveryBehavior
{
public:
    
    // constructor, destructor
    UnleashStaticPlannerRecovery();
    ~UnleashStaticPlannerRecovery();
    
    // initialization of recovery behavior
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

    // run the recovery behavior
    void runBehavior();
    
protected:

    // clients
    actionlib::SimpleActionClient<scitos_2d_navigation::UnleashStaticPlannerAction> *unleash_client;
  
    // ROS handles
    ros::NodeHandle nh;
        
    // recovery behavior stuff
    std::string name_;
    bool initialized_;
    tf::TransformListener* tf_;
    costmap_2d::Costmap2DROS* global_costmap_;
    costmap_2d::Costmap2DROS* local_costmap_;
};
};

#endif // UNLEASH_STATIC_PLANNER_H
