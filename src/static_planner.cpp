#include <scitos_2d_navigation/static_planner.h>
#include <pluginlib/class_list_macros.h>

namespace scitos_2d_navigation
{
// constructor
StaticPlanner::StaticPlanner(std::string name) : unleash_server(nh, name, boost::bind(&StaticPlanner::execute_cb, this, _1), false), action_name(name)
{

        ros::NodeHandle nh("~/");

        //private_nh.param("reset_distance", reset_distance_, 3.0);
        //private_nh.param("layer_search_string", layer_search_string_, std::string("obstacle"));
        ROS_INFO("Start server");
        
        unleash_server.start();

		ROS_INFO("Initialize callbacks and clients");
		
		// flags
		debug = true;
		block_for_block = false;
		finished_process = false;
		got_map = false;
		got_goal = false;
		check_path_is_active = false;
		
		// try...
		//~ tf::StampedTransform transform_stmpd;
		//~ try{
			//~ transform.lookupTransform("map", "base_link", ros::Time(0), transform_stmpd);
			//~ ROS_INFO("Successfully found /map to /base_link tranformation via tf_");
		//~ } catch(tf::TransformException ex) {
			//~ ROS_ERROR("%s",ex.what());
			//~ ros::Duration(1.0).sleep();
		//~ }
		
		//~ ros::Duration(2.0).sleep();
		
		// init the layered costmap
		costmap = new costmap_2d::Costmap2DROS("static_costmap", transform);
		global_costmap_copy = new costmap_2d::Costmap2DROS("global_costmap_copy", transform);
		
		// init the planner with the updating rate
		navfn_plan.initialize("static_planner", costmap);
		loop_rate = new ros::Rate(1.0);
		navfn_check_global.initialize("global_planner_copy", global_costmap_copy);
		
		// init the action clients
		pose_client = new actionlib::SimpleActionClient<perceive_tabletop_action::FindGoalPoseAction>("find_goal_pose", true);
		move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
		
		ROS_INFO("Waiting for action server find_goal_pose to start");
		
		pose_client->waitForServer();
		
		// maybe not neccessary since the move_base is initiating everything anyways?
		//ROS_INFO("Waiting for action server move_base to start");
		//move_base_client->waitForServer();

		ROS_INFO("Finished init for static planner");

		// subscribe to new goals (must be in the very end -> otherwise can jump out of init)
		subGoal = nh.subscribe("/move_base/current_goal", 2, &StaticPlanner::goal_cb, this);
		subDynamicMap = nh.subscribe("/move_base/global_costmap/dynamic_layer/dynamic_map_xxl", 2, &StaticPlanner::dynamic_map_cb, this);

}

// destructor
StaticPlanner::~StaticPlanner()
{
    // void
};

void StaticPlanner::goal_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_WARN("+++ Callback new goal");
    
    // only calculate new path when check_path is not active
    if(!check_path_is_active && got_map) {
		
		// keep it alive in order to keep the path updated
		while(ros::ok() && !check_path_is_active) {
			
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
			
			// one is good enought... right?
			got_goal = true;
			
			ros::spinOnce();    // important not to ignore any callbacks!
			loop_rate->sleep();
		}
	}
};

void StaticPlanner::dynamic_map_cb(const nav_msgs::OccupancyGrid &dynamicMapIn)
{
    if(debug && !got_map)
    ROS_WARN("+++ Received dynamic map");
    
    if(!got_map) {        
        map_height   = dynamicMapIn.info.height;
        map_width    = dynamicMapIn.info.width;
        map_origin_x = dynamicMapIn.info.origin.position.x;
        map_origin_y = dynamicMapIn.info.origin.position.y;
        map_res      = dynamicMapIn.info.resolution;
        mod_num      = (int)(map_res/0.05 + 0.5); // UNSCHÖN! -> solve generically! (0.05 = master_map.resolution...)
        
        ROS_INFO_STREAM("Map width x height: " << map_width << " x " << map_height);
        
        
        // just copy dynamicMap for tableMap and dynMap
        tableMap = new costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);
        dynMap = new costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);
        
        // publisher
        pubTableMap = new costmap_2d::Costmap2DPublisher(&nh, tableMap, "/map", "/table_map", true);        
        pubDynMap = new costmap_2d::Costmap2DPublisher(&nh, dynMap, "/map", "/dynamic_map_xxl_copy", true);
        
        ROS_INFO("Initialized all the maps");
        got_map = true;
        //~ ROS_ERROR("got_map -> true");
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

void StaticPlanner::check_path()
{
	// in order to block out the cb for a new goal
	check_path_is_active = true;
	
	ROS_WARN("+++ Checking path");
    
    for(unsigned int i = 0; i < path.size(); i++) {
        //~ ROS_INFO("path(%d): (%f,%f)", i, path[i].pose.position.x, path[i].pose.position.y);
        
        double pose_x = path[i].pose.position.x;
        double pose_y = path[i].pose.position.y;
        
		// transform to map coordinates (height is in x direction!)
		double map_x = pose_x - map_origin_x;
		double map_y = pose_y - map_origin_y;
		
		// get index on block map
		int i_xxl = map_x / map_res;
		int j_xxl = map_y / map_res;
		
		// get value from map
		int value_map = (int)dynMap->getCost(i_xxl, j_xxl);
		
		//~ ROS_INFO_STREAM("value_map: " << value_map << " - block_for_block: " << block_for_block);
		// check if there is a dynamic obstacle (maybe more agressive over 51?)
		if(value_map > 90 && !block_for_block) {
			// block for the found block
			block_for_block = true;
			
			// warning and map published
			ROS_ERROR_STREAM("Obstacle at (" << map_x+map_origin_x << ", " << map_y+map_origin_y << ") -- Block: (" << i_xxl << ", " << j_xxl << ")  --  Value: " << value_map);
			tableMap->setCost(i_xxl, j_xxl, 200);
			pubTableMap->publishCostmap();
			
			
			// create message to send to simple_view_planner
			perceive_tabletop_action::FindGoalPoseGoal goal;

			// put table into polygon
			goal.polygon.points.resize(4);
			
			// 4 points defining the table polygon
			goal.polygon.points[0].x = map_origin_x + i_xxl*map_res;
			goal.polygon.points[0].y = map_origin_y + j_xxl*map_res;
			goal.polygon.points[0].z = 0;
			goal.polygon.points[1].x = map_origin_x + i_xxl*map_res;
			goal.polygon.points[1].y = map_origin_y + (j_xxl+1)*map_res;
			goal.polygon.points[1].z = 0;
			goal.polygon.points[2].x = map_origin_x + (i_xxl+1)*map_res;
			goal.polygon.points[2].y = map_origin_y + (j_xxl+1)*map_res;
			goal.polygon.points[2].z = 0;
			goal.polygon.points[3].x = map_origin_x + (i_xxl+1)*map_res;
			goal.polygon.points[3].y = map_origin_y + j_xxl*map_res;
			goal.polygon.points[3].z = 0;
			
			try_again:
			
			// send it to the client
			pose_client->sendGoal(goal);
			
			// initialize result
			geometry_msgs::Pose result_pose;

			// wait for the action to return
			bool finished_before_timeout = pose_client->waitForResult(ros::Duration(240.0));
			
			if(finished_before_timeout) {
				
				actionlib::SimpleClientGoalState state = pose_client->getState();
				ROS_INFO("Find goal pose finished: %s",state.toString().c_str());
				result_pose = pose_client->getResult()->goal_pose;
				
				// mold everything to send it to move_base
				geometry_msgs::PoseStamped next_goal;
				
				next_goal.header.stamp = ros::Time::now();
				next_goal.header.frame_id = "map";
				next_goal.pose = result_pose;				
							
				ROS_INFO("Sending Rosie to (x,y,z): (%f, %f, %f)", result_pose.position.x, result_pose.position.y, result_pose.position.z);
				
				bool reachable = pose_is_reachable(next_goal);
				
				// if not successful -> try again
				if(!reachable) {
					ROS_ERROR("Pose not reachable -> try again!");
					goto try_again;
				}
				
				move_base_msgs::MoveBaseAction next_goal_action;
				next_goal_action.action_goal.goal.target_pose = next_goal;
				
				ROS_INFO("Are we connected?");
				//~ actionlib::SimpleClientGoalState state_move_base = move_base_client->getState();
				bool state_move_base_connected = move_base_client->isServerConnected();
				ROS_INFO_STREAM("Connected: " << state_move_base_connected);
				//~ ROS_INFO("Situation before cancelling");
				//~ actionlib::SimpleClientGoalState state_move_base = move_base_client->getState();
				//~ ROS_INFO("Move base client: %s", state_move_base.toString().c_str());
				//~ 
				// send it to move_base
				ROS_INFO("Cancel all goals");
				move_base_client->cancelAllGoals();
				//~ move_base_client->waitForResult(ros::Duration(10.0));
				//~ state_move_base = move_base_client->getState();
				//~ ROS_INFO("Move base client: %s", state_move_base.toString().c_str());
				
				ROS_INFO("Send new goal");
				move_base_client->sendGoalAndWait(next_goal_action.action_goal.goal, ros::Duration(120.0), ros::Duration(10.0));
				actionlib::SimpleClientGoalState state_move_base = move_base_client->getState();
				ROS_INFO("Move base client: %s", state_move_base.toString().c_str());
				move_base_client->waitForResult(ros::Duration(120.0));
				
				if(move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
					ROS_INFO("Moved into position!");
				} else {
					ROS_INFO("Move failed!");
				}
				
				// free up block_for_block
				block_for_block = false;

			} else {
				ROS_INFO("Find goal pose not finish before the time out.");
			}
			

			
			// save original next goal
			// send new goal to the nav_goal
			// check if reached the goal
			// set back original goal
			// block_for_block = false

		}
    }
    
    // free up cb for new goals again
    check_path_is_active = false;
    
    ROS_INFO("Finished checking path");
}

bool StaticPlanner::pose_is_reachable(const geometry_msgs::PoseStamped &check_pose)
{
	geometry_msgs::PoseStamped tmsg;

	ros::Time now = ros::Time::now();
	transform.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
	transform.transformPose("/map", check_pose, tmsg);
	geometry_msgs::PoseStamped start = tmsg, tstart;
	start.header.frame_id = "/base_link";
	start.pose.position.x = 0;
	start.pose.position.y = 0;
	start.pose.position.z = 0;
	transform.transformPose( "/map", start, tstart );
	
	std::vector<geometry_msgs::PoseStamped> global_path;
	bool plan_found_in_global = navfn_check_global.makePlan(tstart, tmsg, global_path);
	if(!plan_found_in_global) {
		ROS_WARN("Goal not reachable");
		return false;
	} else {
		ROS_WARN("Goal reachable");
		return true;
	}
};



void StaticPlanner::execute_cb(const scitos_2d_navigation::UnleashStaticPlannerGoalConstPtr &goal)
{
	ROS_ERROR("Executing action server call");
	//~ subDynamicMap.shutdown();
	//~ subGoal.shutdown();
	check_path();
	unleash_server.setSucceeded();
};
};

int main( int argc, char* argv[] )
{
    
    ros::init(argc, argv, "static_planner");
    
    scitos_2d_navigation::StaticPlanner unleash_static_planner(ros::this_node::getName());

    ros::spin();
    return 0;
}
