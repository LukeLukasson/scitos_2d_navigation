#include <scitos_2d_navigation/static_planner.h>
#include <pluginlib/class_list_macros.h>

namespace scitos_2d_navigation
{
// constructor
StaticPlanner::StaticPlanner(std::string name) : unleash_server(nh, name, boost::bind(&StaticPlanner::execute_cb, this, _1), false), action_name(name)
{

        ros::NodeHandle nh("~/");

        ROS_INFO("Start server");
        
        unleash_server.start();

		ROS_INFO("Initialize callbacks and clients");
		
		// flags
		debug = true;
		block_for_block = false;
		got_map = false;
		got_goal = false;
		check_path_is_active = false;
		
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
    
    // only calculate new path when check_path is not active
    if(!check_path_is_active && got_map) {
    ROS_WARN("+++ Callback new goal");
		
		// keep it alive in order to keep the path updated
		while(ros::ok() && !check_path_is_active) {
			
			// core of the planning algorithm
			geometry_msgs::PoseStamped tmsg;

			transform.transformPose("/map", *msg, tmsg);
			
			// store original goal -> will not be overwritten due to the check_path_is_active protection
			original_goal = tmsg;
			
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
    if(!got_map)
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
		if(value_map > 90) {
		
			// update critical pose
			crit_dyn_obs.pose.position.x = map_x;
			crit_dyn_obs.pose.position.y = map_y;
			crit_dyn_obs.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			
			// warning and map published
			ROS_ERROR_STREAM("Obstacle at (" << map_x+map_origin_x << ", " << map_y+map_origin_y << ") -- Block: (" << i_xxl << ", " << j_xxl << ")  --  Value: " << value_map);
			tableMap->setCost(i_xxl, j_xxl, 200);
			pubTableMap->publishCostmap();
			
			// create message to send to simple_view_planner
			perceive_tabletop_action::FindGoalPoseGoal observation_polygon;

			// put table into polygon
			observation_polygon.polygon.points.resize(4);
			
			// 4 points defining the table polygon
			observation_polygon.polygon.points[0].x = map_origin_x + i_xxl*map_res;
			observation_polygon.polygon.points[0].y = map_origin_y + j_xxl*map_res;
			observation_polygon.polygon.points[0].z = 0;
			observation_polygon.polygon.points[1].x = map_origin_x + i_xxl*map_res;
			observation_polygon.polygon.points[1].y = map_origin_y + (j_xxl+1)*map_res;
			observation_polygon.polygon.points[1].z = 0;
			observation_polygon.polygon.points[2].x = map_origin_x + (i_xxl+1)*map_res;
			observation_polygon.polygon.points[2].y = map_origin_y + (j_xxl+1)*map_res;
			observation_polygon.polygon.points[2].z = 0;
			observation_polygon.polygon.points[3].x = map_origin_x + (i_xxl+1)*map_res;
			observation_polygon.polygon.points[3].y = map_origin_y + j_xxl*map_res;
			observation_polygon.polygon.points[3].z = 0;
			
			// create pose for checking if in sight of robot
			crit_dyn_obs.header.frame_id = "map";
			crit_dyn_obs.pose.position.x = pose_x;
			crit_dyn_obs.pose.position.y = pose_y;
			crit_dyn_obs.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
			
			
			/* 
			 * start the logic of the recovery behavior
			 */
			 
			// does Rosie need to move?
			if(check_if_in_sight(crit_dyn_obs)) {
				// we do not have to move:
				// check if original goal reachable
				if(pose_is_reachable(original_goal)) {
					// continue journey of Rosie and check if succeeded
					if(send_rosie_to_original_goal()) {
						// no recovery needed
						ROS_INFO("Recovery succeeded without interaction");
						sound_play::Sound success_sound = sound_client.voiceSound("Who needs this shitty algorithm anyway?");
						success_sound.play();					
					} else {
						// recovery failed
						ROS_WARN("Recovery failed even though original goal would be reachable");
						sound_play::Sound failed_sound = sound_client.voiceSound("I have a valid path but cannot reach the original goal.");
						failed_sound.play();
					}
				} else {
					// original pose not reachable. proably due to the dynamic obstacle in sight
					// interact with obstacle and check if succeeded
					if(interact_and_check()) {
						// dynamic obstacle moved -> Go to original target
						sound_play::Sound thanks_sound = sound_client.voiceSound("Thanks for helping me.");
						thanks_sound.play();
						if(send_rosie_to_original_goal()) {
							// recovery successful
							ROS_INFO("Recovery succeeded with interaction");
							sound_play::Sound success_sound = sound_client.voiceSound("I made it! Booooyyaaaah!");
							success_sound.play();
						} else {
							// recovery failed
							ROS_WARN("Recovery failed after interaction");
							sound_play::Sound failed_sound = sound_client.voiceSound("I have a valid path but cannot reach the original goal.");
							failed_sound.play();
						}
					} else {
						// dynamic obstacle did not move
						ROS_WARN("Recovery failed since the dynamic obstacle did not move out of the way");
						sound_play::Sound failed_sound = sound_client.voiceSound("If you dont want to move then I do not want to move either.");
						failed_sound.play();					
					}
				}	
			} else {
				// we do need to move
				if(run_mobile_reobservation(observation_polygon)) {
					// successfully moved into reobservation pose -> did it solve the problem?
					if(pose_is_reachable(original_goal)) {
						// continue journey of Rosie and check if succeeded
						if(send_rosie_to_original_goal()) {
							// no recovery needed
							ROS_INFO("Recovery succeeded without interaction after reobserving obstacle");
							sound_play::Sound success_sound = sound_client.voiceSound("No interaction needed. Stupid humans.");
							success_sound.play();
						} else {
							// recovery failed
							ROS_WARN("Recovery failed even though original goal would be reachable after reobserving  obstacle");
							sound_play::Sound failed_sound = sound_client.voiceSound("I have a valid path but cannot reach the original goal.");
							failed_sound.play();
						}
					} else {
						// original pose not reachable. proably due to the dynamic obstacle in sight
						// interact with obstacle and check if succeeded
						if(interact_and_check()) {
							// dynamic obstacle moved -> Go to original target
							sound_play::Sound thanks_sound = sound_client.voiceSound("Thanks for helping me.");
							thanks_sound.play();
							if(send_rosie_to_original_goal()) {
								// recovery successful
								ROS_INFO("Recovery succeeded with interaction after reobserving obstacle");
								sound_play::Sound success_sound = sound_client.voiceSound("I made it! Booooyyaaaah!");
								success_sound.play();
							} else {
								// recovery failed
								ROS_WARN("Recovery failed after interaction after reobserving obstacle");
							}
						} else {
							// dynamic obstacle did not move
							ROS_WARN("Recovery failed since the dynamic obstacle did not move out of the way");
							sound_play::Sound failed_sound = sound_client.voiceSound("If you dont want to move then I do not want to move either.");
							failed_sound.play();
						}
					}	
				} else {
					// cannot reach reobservation pose
					ROS_WARN("Recovery failed. Could not evaluate or reach a valid pose.");
					sound_play::Sound failed_sound = sound_client.voiceSound("Cannot find any pose for reobserving the dynamic obstacle.");
					failed_sound.play();		
				}
			}
			
			// break out of the for loop
			break;
		}
	}

	// clear check_path() for new execution
    check_path_is_active = false;
    
    ROS_INFO("Finished recovery behavior static planner");
};			
			
// true: pose reachable on modified copy of global_costmap - false: anything else
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
	return plan_found_in_global;
	//~ if(!plan_found_in_global) {
		//~ ROS_WARN("  Pose not reachable");
		//~ return false;
	//~ } else {
		//~ ROS_INFO("  Pose reachable");
		//~ return true;
	//~ }
};

// true: dynamic point (!) is within rectangle in front of Rosie - false: anything else
bool StaticPlanner::check_if_in_sight(const geometry_msgs::PoseStamped &target)
{
	geometry_msgs::PoseStamped target_rosie;
	
	ros::Time now = ros::Time::now();
	transform.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
	transform.transformPose("/base_link", target, target_rosie);
	
	double x = target_rosie.pose.position.x;
	double y = target_rosie.pose.position.y;
	
	ROS_INFO_STREAM("  Target as seen from Rosie (x,y): (" << x << "," << y << " - Check if in sight:");

	if((x > 0 && x < 2.0) && (y > -0.35 && y < 0.35)) {
		ROS_INFO("  Target in sight. Do not need to move");
		return true;
	} else {
		ROS_WARN("  Target not in sight. Do need to reposition");
		return false;
	}
};

// true: reached goal - false: anything else
bool StaticPlanner::send_rosie_to_original_goal()
{
	move_base_msgs::MoveBaseAction original_goal_action;
	original_goal_action.action_goal.goal.target_pose = original_goal;
	move_base_client->sendGoalAndWait(original_goal_action.action_goal.goal, ros::Duration(120.0), ros::Duration(10.0));
	if(move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("  Successfully reached original goal");
		return true;
	} else {
		ROS_WARN("  Could not reach original goal");
		return false;
	}
};

// true: reached goal - false: anyting else
bool StaticPlanner::send_rosie_to_pose(const geometry_msgs::PoseStamped &pose)
{
	move_base_msgs::MoveBaseAction send_rosie_action;
	send_rosie_action.action_goal.goal.target_pose = pose;
	move_base_client->sendGoalAndWait(send_rosie_action.action_goal.goal, ros::Duration(120.0), ros::Duration(10.0));
	if(move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("  Successfully reached desired pose");
		return true;
	} else {
		ROS_WARN("  Could not reach desired pose");
		return false;
	}
		
};

// true: original goal reachable - false: anything else
bool StaticPlanner::interact_and_check()
{
	// interact with dynamic obstacle
	ROS_INFO("  Starting interaction...");
	sound_play::Sound ask_sound = sound_client.voiceSound("Hello dynamic obstacle. Please move out of the way.");
	ask_sound.play();
	bool original_goal_reachable = false;
	// try for 20 seconds -> then give up
	int abort_counter = 0;
	while(!original_goal_reachable && abort_counter <= 80) {
		ros::Duration(0.25).sleep();
		abort_counter++;
		original_goal_reachable = pose_is_reachable(original_goal);
	}
	return original_goal_reachable;
};

// true: successfully reached a pose to reobserve the dynamic obstacle - false: any error
bool StaticPlanner::run_mobile_reobservation(const perceive_tabletop_action::FindGoalPoseGoal &observation_polygon)
{
	// upper limits for time to execute
	ros::Time start_time = ros::Time::now();
	bool observation_pose_reachable = false;
	bool reached_observation_pose = false;
	
	// init observation pose
	geometry_msgs::PoseStamped observation_pose;
	observation_pose.header.frame_id = "map";
	
	// outer loop: 30s to move into obervation pose
	while(!reached_observation_pose && (ros::Time::now()-start_time < ros::Duration(30))) {
		
		// inner loop: 10s to find a valid observation pose
		while(!observation_pose_reachable && (ros::Time::now()-start_time < ros::Duration(10))) {
			
			// send polygon to pose evaluation client
			pose_client->sendGoal(observation_polygon);
			bool finished_before_timeout = pose_client->waitForResult(ros::Duration(5.0));
			ROS_INFO_STREAM("  Finished table top part before time out: " << finished_before_timeout);
			
			if(pose_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				// successuflly found a pose -> is it reachable?
				observation_pose.pose = pose_client->getResult()->goal_pose;
				observation_pose.header.stamp = ros::Time::now();
				observation_pose_reachable = pose_is_reachable(observation_pose);
				ROS_INFO_STREAM("  Result if (" << observation_pose.pose.position.x << "," << observation_pose.pose.position.y << ") is reachable: " << observation_pose_reachable);
			}
		}
		
		if(observation_pose_reachable) {
			// cancel all goals and send Rosie to observation pose
			// Did that ever fail? Not really...  bool state_move_base_connected = move_base_client->isServerConnected();			
			reached_observation_pose = send_rosie_to_pose(observation_pose);
		}
	}
	
	return reached_observation_pose;
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
