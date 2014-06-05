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
    
    // public check for occupied path
    void check_path(); // needs tons of exception handles!!!
    
protected:

    // callback for goal subscriber
    void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
    void dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamic_map_in);
    
    // ROS handles
    ros::NodeHandle nh;
    ros::Rate *loop_rate;
    
    // Subscribers
    ros::Subscriber subGoal;
    ros::Subscriber subDynamicMap;
    
    // maps
    nav_msgs::OccupancyGrid dynamicMap;
    bool init_map;
    double map_width;
    double map_height;
    double map_origin_x;
    double map_origin_y;
    double map_res;
    int mod_num;
    
    // listen to tf
    tf::TransformListener transform;
    
    // init
    costmap_2d::Costmap2DROS *costmap;
    navfn::NavfnROS navfn_plan;
    
    // make path available globally
    std::vector<geometry_msgs::PoseStamped> path;
    
    // counters
    int cb_counter;
};

// constructor
static_planner_node::static_planner_node():
    nh("~")
{
    ROS_WARN("Initialize node");
    
    // subscribe to new goals
    subGoal = nh.subscribe("/move_base/current_goal", 2, &static_planner_node::goal_cb, this);
    subDynamicMap = nh.subscribe("/move_base/global_costmap/dynamic_layer/dynamic_map_xxl", 2, &static_planner_node::dynamic_map_cb, this);
    
    // init the layered costmap
    costmap = new costmap_2d::Costmap2DROS("static_costmap", transform);
    
    // init the planner with the updating rate
    navfn_plan.initialize("static_planner", costmap);
    loop_rate = new ros::Rate(5.0);
    
    // flags
    init_map = false;
    
    // counters
    cb_counter = 0;
    
};

// destructor
static_planner_node::~static_planner_node()
{
    // void
};
    
void static_planner_node::goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_WARN("+++ Callback new goal");
    
    while(ros::ok()) {
        
        // core of the planning algorithm
        geometry_msgs::PoseStamped tmsg;

        transform.transformPose("/map", *msg, tmsg);
        geometry_msgs::PoseStamped start = tmsg, tstart;
        start.header.frame_id = "/base_link";
        start.pose.position.x = 0;
        start.pose.position.y = 0;
        start.pose.position.z = 0;
        transform.transformPose( "/map", start, tstart );

        //~ ROS_INFO("plan(%s): (%f,%f) ==> (%f,%f)", tmsg.header.frame_id.c_str(), tstart.pose.position.x,tstart.pose.position.y,tmsg.pose.position.x,tmsg.pose.position.y);


        bool planned = navfn_plan.makePlan(tstart, tmsg, path);
        if(!planned) {
            ROS_WARN("plan did not succeed");
            }

        navfn_plan.publishPlan(path, 0.0, .8, 0.0, 0.2);
        
        // check if any blocks are on the planned way
        check_path();
        
        ros::spinOnce();    // important not to ignore any callbacks!
        loop_rate->sleep();
    }
};

void static_planner_node::dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamic_map_in)
{

    ROS_WARN("+++ Received dynamic map");
    
    // copy process every time. Pointer would be better -> something for later...
    // maybe dynamicMpa->resolution...
    dynamicMap = dynamic_map_in;
    cb_counter = 0;
    
    if(!init_map) {
        map_width   = dynamicMap.info.height;
        map_height  = dynamicMap.info.width;
        map_origin_x= dynamicMap.info.origin.position.x;
        map_origin_y= dynamicMap.info.origin.position.y;
        map_res     = dynamicMap.info.resolution;
        mod_num     = (int)(map_res/0.05 + 0.5); // UNSCHÖN! -> solve generically! (0.05 = master_map.resolution...)
        
        init_map = true;
    }
    
};

void static_planner_node::check_path()
{
    ROS_WARN("+++ Checking path");
    
    for(unsigned int i = 0; i < path.size(); i++) {
        //~ ROS_INFO("path(%d): (%f,%f)", i, path[i].pose.position.x, path[i].pose.position.y);
        
        double pose_x = path[i].pose.position.x;
        double pose_y = path[i].pose.position.y;
        
        if(init_map) {
            
            // transform to map coordinates (height is in x direction!)
            double map_x = pose_x - map_origin_x;
            double map_y = pose_y - map_origin_y;
            
            // get index on block map
            int i_xxl = map_x / map_res;
            int j_xxl = map_y / map_res;
            
            // get value from map
            int value_map = (int)dynamicMap.data[j_xxl*map_height + i_xxl];
            
            //~ ROS_INFO_STREAM("width:  " << map_width << " res: " << map_res << " Number of cells: " << map_width*map_res << " " << map_width/map_res << " " << mod_num);
            //~ ROS_INFO_STREAM("height: " << map_height << " res: " << map_res << " Number of cells: " << map_height*map_res << " " << map_height/map_res);
            //~ 
            //~ for(int j = 0; j < 60; j++) {
                //~ ROS_WARN_STREAM("Data value at " << j << " is " << (int)dynamicMap.data[j]);
            //~ }
            //~ ROS_WARN_STREAM("Map: (" << map_x << ", " << map_y << ")  --  Square: (" << i_xxl << ", " << j_xxl << ")  --  Value: " << value_map);
            
            if(value_map > 90) {
                ROS_ERROR_STREAM("Obstacle at (" << map_x+map_origin_x << ", " << map_y+map_origin_y << ") -- Block: (" << i_xxl << ", " << j_xxl << ")  --  Value: " << value_map);
            }
        }
    }
}

int main( int argc, char* argv[] )
{
    
    ros::init(argc, argv, "static_planner");
    static_planner_node x;

    ros::spin();
    return 0;
}
