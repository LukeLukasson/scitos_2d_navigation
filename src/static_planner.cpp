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

void static_planner_node::dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamicMapIn)
{

    ROS_WARN("+++ Received dynamic map");
    
    if(!init_map) {        
        map_height   = dynamicMapIn.info.height;
        map_width    = dynamicMapIn.info.width;
        map_origin_x = dynamicMapIn.info.origin.position.x;
        map_origin_y = dynamicMapIn.info.origin.position.y;
        map_res      = dynamicMapIn.info.resolution;
        mod_num      = (int)(map_res/0.05 + 0.5); // UNSCHÖN! -> solve generically! (0.05 = master_map.resolution...)
        
        ROS_INFO_STREAM("width: " << map_width << " --- height: " << map_height);
        
        init_map = true;
        
        // just copy dynamicMap for visMap
        tableMap = new costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);
        dynMap = new costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);
        
        // publisher
        pubTableMap = new costmap_2d::Costmap2DPublisher(&nh, tableMap, "/map", "/table_map", true);        
        pubDynMap = new costmap_2d::Costmap2DPublisher(&nh, dynMap, "/map", "/dynamic_map_xxl_copy", true);        

    }
    
    // incoming dynamic map to Costmap2D
    for(int i=0; i<map_width; i++) {
        for(int j=0; j<map_height; j++) {
            dynMap->setCost(i, j, dynamicMapIn.data[dynMap->getIndex(i,j)]);
        }
    }
    
    pubTableMap->publishCostmap();
    pubDynMap->publishCostmap();
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
            int value_map = (int)dynMap->getCost(i_xxl, j_xxl);
            
            // check if there is a dynamic obstacle (maybe more conservative over 51?)
            if(value_map > 90) {
                ROS_ERROR_STREAM("Obstacle at (" << map_x+map_origin_x << ", " << map_y+map_origin_y << ") -- Block: (" << i_xxl << ", " << j_xxl << ")  --  Value: " << value_map);
                tableMap->setCost(i_xxl, j_xxl, 200);
                pubTableMap->publishCostmap();
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
