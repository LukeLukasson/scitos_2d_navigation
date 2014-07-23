#include <scitos_2d_navigation/unleash_static_planner.h>
#include <pluginlib/class_list_macros.h>


//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(scitos_2d_navigation, UnleashPlannerRecovery, scitos_2d_navigation::UnleashStaticPlannerRecovery, nav_core::RecoveryBehavior)

namespace scitos_2d_navigation
{
// constructor
UnleashStaticPlannerRecovery::UnleashStaticPlannerRecovery() : initialized_(false)
{
    // void
}
    
void UnleashStaticPlannerRecovery::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    if(!initialized_){
        name_ = name;
        tf_ = tf;
        global_costmap_ = global_costmap;
        local_costmap_ = local_costmap;

        //get some parameters from the parameter server
        ros::NodeHandle nh("~/" + name_);


		// init the action clients
		unleash_client = new actionlib::SimpleActionClient<scitos_2d_navigation::UnleashStaticPlannerAction>("static_planner", true);

		ROS_INFO("Waiting for static_planner action server to start.");
		
		unleash_client->waitForServer();
		
        initialized_ = true;
        
    }
    else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
};

// destructor
UnleashStaticPlannerRecovery::~UnleashStaticPlannerRecovery()
{
    // void
};

// Run behavior for recovery
void UnleashStaticPlannerRecovery::runBehavior()
{
    ROS_WARN("OK! Let's go! Unleash the static planner!");
    
    scitos_2d_navigation::UnleashStaticPlannerGoal goal;
    goal.execute = true;
    
    ROS_INFO("Sending execute command to the static planner.");
    unleash_client->sendGoal(goal);
    
    ROS_INFO("Done deal. Recovery behavior should exit now.");
}
    
};
