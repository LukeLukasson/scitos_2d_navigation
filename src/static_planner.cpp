#include <scitos_2d_navigation/static_planner.h>

// constructor
static_planner_node::static_planner_node():
    nh("~")
{
    ROS_WARN("Initialize node");
    
    // flags
    init_map = false;    
    
    // handles the datacentre connection
    ROS_INFO("Here fine");
    ros_datacentre::MessageStoreProxy messageStore(nh);
    
    ROS_INFO("Here fine");
    strands_perception_msgs::Table my_table;
    my_table.table_id = "MagicTable1";
    my_table.header.frame_id = "/map";
    
    my_table.pose.pose.position.x = 0;
    my_table.pose.pose.position.y = 0;
    my_table.pose.pose.position.z = 0;
    
    my_table.pose.pose.orientation.x = 0;
    my_table.pose.pose.orientation.y = 0;
    my_table.pose.pose.orientation.z = 0;
    my_table.pose.pose.orientation.w = 1;
    
    ROS_INFO("Here fine");
    // adding 4 points
    my_table.tabletop.points.resize(4);

    my_table.tabletop.points[0].x = 0;
    my_table.tabletop.points[0].y = 0;
    my_table.tabletop.points[0].z = 0;
    
    my_table.tabletop.points[1].x = 1;
    my_table.tabletop.points[1].y = 0;
    my_table.tabletop.points[1].z = 0;
    
    my_table.tabletop.points[2].x = 1;
    my_table.tabletop.points[2].y = 1;
    my_table.tabletop.points[2].z = 0;
    
    my_table.tabletop.points[3].x = 0;
    my_table.tabletop.points[3].y = 1;
    my_table.tabletop.points[3].z = 0;

    ROS_INFO("Here fine");
    std::string id(messageStore.insertNamed("MagicTable", my_table));
    ROS_INFO_STREAM("Table with id " << id << " inserted.");
    
    
        
 
    // init the layered costmap
    costmap = new costmap_2d::Costmap2DROS("static_costmap", transform);
    
    // init the planner with the updating rate
    navfn_plan.initialize("static_planner", costmap);
    loop_rate = new ros::Rate(5.0);
    

    ROS_INFO("Finished init");
    
    // subscribe to new goals (must be in the very end -> otherwise can jump out of init)
    subGoal = nh.subscribe("/move_base/current_goal", 2, &static_planner_node::goal_cb, this);
    subDynamicMap = nh.subscribe("/move_base/global_costmap/dynamic_layer/dynamic_map_xxl", 2, &static_planner_node::dynamic_map_cb, this);
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
