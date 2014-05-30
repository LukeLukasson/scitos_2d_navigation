#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <geometry_msgs/PoseStamped.h>

// class header
class static_planner_node
{
public:
    
    // constructor, destructor
    static_planner_node();
    ~static_planner_node();
    
protected:
    
    // callback for goal subscriber
    void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    
    // ROS handles
    ros::NodeHandle nh;
    ros::Rate *loop_rate;
    // Subscribers
    ros::Subscriber subGoal;

    // listen to tf
    tf::TransformListener transform;
    
    costmap_2d::Costmap2DROS *costmap;
    navfn::NavfnROS navfn_plan;
    
    // make path available globally
    std::vector<geometry_msgs::PoseStamped> path;
};

// constructor
static_planner_node::static_planner_node():
    nh("~")
{
    // subscribe to new goals
    subGoal = nh.subscribe("/move_base/current_goal", 2, &static_planner_node::goal_cb, this);
    
    // init the layered costmap
    costmap = new costmap_2d::Costmap2DROS("static_costmap", transform);
    
    // init the planner with the updating rate
    navfn_plan.initialize("static_planner", costmap );
    loop_rate = new ros::Rate(20.0);
    
};

// destructor
static_planner_node::~static_planner_node()
{
    // void
};
    
void static_planner_node::goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_INFO("Callback new goal");
    
    while(ros::ok()) {
        
        // core of the planning algorithm
        geometry_msgs::PoseStamped tmsg;

        transform.transformPose( "/map", *msg, tmsg );
        geometry_msgs::PoseStamped start = tmsg, tstart;
        start.header.frame_id = "/base_link";
        start.pose.position.x = 0;
        start.pose.position.y = 0;
        start.pose.position.z = 0;
        transform.transformPose( "/map", start, tstart );

        ROS_INFO( "plan(%s): (%f,%f) ==> (%f,%f)", tmsg.header.frame_id.c_str(), tstart.pose.position.x,tstart.pose.position.y,tmsg.pose.position.x,tmsg.pose.position.y );


        bool planned = navfn_plan.makePlan( tstart, tmsg, path );
        if( !planned )
            {
            ROS_WARN( "plan did not succeed" );
            }
        /*
        for( unsigned int i = 0; i < path.size(); i++ )
            {
            ROS_INFO( "path(%d): (%f,%f)", i, path[i].pose.position.x, path[i].pose.position.y );
            }
        */
        navfn_plan.publishPlan( path, 0.0, .8, 0.0, 0.2 );
        
        ros::spinOnce();    // important not to ignore any callbacks!
        loop_rate->sleep();
    }
}


int main( int argc, char* argv[] )
{
    
    ros::init(argc, argv, "static_planner");
    static_planner_node x;

    ros::spin();
    return 0;
}
