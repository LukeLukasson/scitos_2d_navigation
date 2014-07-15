#include <scitos_2d_navigation/dynamic_layer.h>
#include <pluginlib/class_list_macros.h>

#include "std_msgs/String.h"
#include <vector>               // needed for updating map
#include <map>                  // needed for converting Eigen matrix to array
#include <Eigen/StdVector>      // needed for converting Eigen matrix to array
#include <sys/types.h>          // for getting int8_t

PLUGINLIB_EXPORT_CLASS(scitos_2d_navigation::DynamicLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

// Luke
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::VectorXi;

/* ToDo
 * ====
 * 
 * o Clean up code
 * o Check SUPER HACK things
 * 
 * Done
 * ====
 * 
 * x Parameters by reference in functions? Why better? -> see schlachtfeld
 * x initStaticMap does not need an argument!!!
 */
 
 
namespace scitos_2d_navigation
{

// Destructor
DynamicLayer::~DynamicLayer()
{
    // void
};

void DynamicLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    // configure dynamic parameters
    dsrv_ = new dynamic_reconfigure::Server<scitos_2d_navigation::dynamic_layer_paramsConfig>(nh);
    dynamic_reconfigure::Server<scitos_2d_navigation::dynamic_layer_paramsConfig>::CallbackType cb = boost::bind(
      &DynamicLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // flags
    flag_init = false;
    flag_init_fine = false;
    flag_init_block = false;
    flag_init_input = false;
    nh.param("debug_flag", debug, false);
    nh.param("init_static_map_blank", init_fine_blank, false);
    nh.param("publish_fine_map", publish_fine_map, false);
    nh.param("publish_block_map", publish_block_map, false);
    nh.param("publish_input_map", publish_input_map, false);
    
    // assign seq numbers to maps
    staticMap_seq = 1;
    staticMap_xxl_seq = 2;
    dynamicMap_seq = 3;
    dynamicMap_xxl_seq = 4;
    if(publish_input_map) inputMap_seq = 5;
    
    // parameters algorithm
    lower_bound = 0.1;                            // free space below 0.1 prob
    upper_bound = 0.9;                            // occupied space over 0.9 prob
    
    // map parameters
    nh.param("resolution_subsampling", resolution_xxl, 0.2);
    nh.param("max_cells_x", max_cells_x, 100);
    nh.param("max_cells_y", max_cells_y, 100);
    
    // screws up algorithm if value goes to 0.0 or 1.0
    double map_min_value_server, map_max_value_server; // only double on servers...
    nh.param("map_min_value", map_min_value_server, 0.001);
    nh.param("map_max_value", map_max_value_server, 0.999);
    map_min_value = (float)map_min_value_server;
    map_max_value = (float)map_max_value_server;
    
    nh.param("stat_low", stat_low, 0.4);        // how fast shall it vanish?
    stat_Low = stat_low/(1-stat_low);
    nh.param("stat_low2", stat_low2, 0.52);        // introduce static dynamic obstacle into static map
    stat_Low2 = stat_low2/(1-stat_low2);
    nh.param("stat_high", stat_high, 0.9);        // how fast shall we believe?
    stat_High = stat_high/(1-stat_high);

    
    nh.param("dyn_low", dyn_low, 0.2);            // how fast shall it vanish?
    dyn_Low = dyn_low/(1-dyn_low);
    nh.param("dyn_high", dyn_high, 0.99);        // how fast shall we believe?
    dyn_High = dyn_high/(1-dyn_high);
    
    if(debug)
    ROS_INFO_STREAM("Settings -- stat_low: " << stat_low << " -- stat_low2: " << stat_low2 << " -- stat_high: " << stat_high << " -- dyn_low: " << dyn_low << " -- dyn_high: " << dyn_high);
    
}

// initialize static map
void DynamicLayer::initStaticMap()
{    
    if(debug) ROS_WARN("+++ Initializing static map");
    
    // handeling nav_msgs/MapMetaData
    staticMap.info.resolution = master_grid_resolution;                                                                 // float32
    staticMap.info.width = width;                                                      // uint32
    staticMap.info.height = height;                                                     // uint32
    staticMap.info.origin.position.x = std::max(-(max_cells_x/2 * master_grid_resolution), master_grid_origin_x);         // same origin as /map
    staticMap.info.origin.position.y = std::max(-(max_cells_y/2 * master_grid_resolution), master_grid_origin_y);         // same origin as /map
    staticMap.info.origin.orientation.w = master_grid_orientation;                                                      // same orientation as /map
    int p[n_cells];
    std::vector<signed char> a(p, p+n_cells);
    staticMap.data = a;

    
    // get initializer
    staticMap.header.seq = staticMap_seq;
}

// initialize static map xxl
void DynamicLayer::initStaticMapXxl()
{
    if(debug) ROS_WARN("+++ Initializing static map xxl");
    
    // handeling nav_msgs/MapMetaData
    staticMap_xxl.info.resolution = resolution_xxl;                     // float32
    staticMap_xxl.info.width = width_xxl;                               // uint32
    staticMap_xxl.info.height = height_xxl;                             // uint32
    staticMap_xxl.info.origin.position.x = std::max(-(max_cells_x/2 * master_grid_resolution), master_grid_origin_x);       // same origin as /map
    staticMap_xxl.info.origin.position.y = std::max(-(max_cells_y/2 * master_grid_resolution), master_grid_origin_y);      // same origin as /map
    staticMap_xxl.info.origin.orientation.w = master_grid_orientation;                      // same orientation as /map
    int p[n_cells_xxl];
    std::vector<signed char> a(p, p+n_cells_xxl);
    staticMap_xxl.data = a;
    
    // get initializer
    staticMap_xxl.header.seq = staticMap_xxl_seq;
}

// initialize dynamic map
void DynamicLayer::initDynamicMap()
{
    if(debug) ROS_WARN("+++ Initializing dynamic map");
    
    // handeling nav_msgs/MapMetaData
    dynamicMap.info.resolution = master_grid_resolution;                                                                // float32
    dynamicMap.info.width = width;                                                     // uint32
    dynamicMap.info.height = height;                                                    // uint32
    dynamicMap.info.origin.position.x = std::max(-(max_cells_x/2 * master_grid_resolution), master_grid_origin_x);        // same origin as /map
    dynamicMap.info.origin.position.y = std::max(-(max_cells_y/2 * master_grid_resolution), master_grid_origin_y);        // same origin as /map
    dynamicMap.info.origin.orientation.w = master_grid_orientation;                                                     // same orientation as /map
    int p[n_cells];
    std::vector<signed char> a(p, p+n_cells);
    dynamicMap.data = a;
    
    // get initializer
    dynamicMap.header.seq = dynamicMap_seq;
}

// initialize dynamic map xxl
void DynamicLayer::initDynamicMapXxl()
{
    if(debug) ROS_WARN("+++ Initializing dynamic map xxl");
    
    // handeling nav_msgs/MapMetaData
    dynamicMap_xxl.info.resolution = resolution_xxl;                     // float32
    dynamicMap_xxl.info.width = width_xxl;                               // uint32
    dynamicMap_xxl.info.height = height_xxl;                             // uint32
    dynamicMap_xxl.info.origin.position.x = std::max(-(max_cells_x/2 * master_grid_resolution), master_grid_origin_x);       // same origin as /map
    dynamicMap_xxl.info.origin.position.y = std::max(-(max_cells_y/2 * master_grid_resolution), master_grid_origin_y);      // same origin as /map
    dynamicMap_xxl.info.origin.orientation.w = master_grid_orientation;                      // same orientation as /map
    int p[n_cells_xxl];
    std::vector<signed char> a(p, p+n_cells_xxl);
    dynamicMap_xxl.data = a;
    
    // get initializer
    dynamicMap_xxl.header.seq = dynamicMap_xxl_seq;
}

// initialize input map
void DynamicLayer::initInputMap()
{    
    if(debug) ROS_WARN("+++ Initializing input map");
    
    // handeling nav_msgs/MapMetaData
    inputMap.info.resolution = master_grid_resolution;                                                                 // float32
    inputMap.info.width = width;                                                      // uint32
    inputMap.info.height = height;                                                     // uint32
    inputMap.info.origin.position.x = std::max(-(max_cells_x/2 * master_grid_resolution), master_grid_origin_x);         // same origin as /map
    inputMap.info.origin.position.y = std::max(-(max_cells_y/2 * master_grid_resolution), master_grid_origin_y);         // same origin as /map
    inputMap.info.origin.orientation.w = master_grid_orientation;                                                      // same orientation as /map
    int p[n_cells];
    std::vector<signed char> a(p, p+n_cells);
    inputMap.data = a;
    
    // get initializer
    inputMap.header.seq = inputMap_seq;
}

// push values of matrix to OccupancyGrid of map
void DynamicLayer::publishMap(nav_msgs::OccupancyGrid &map, Eigen::MatrixXf &matrix, int cells)
{
   
    // how many cells in map? -> n_cells
    // cast <float> matrix to <int> matrix
    //~ MatrixXf matrix_copy = 100*matrix;    
    MatrixXi matrix_int = (matrix*100).cast<int>();
    
    // transform Eigen::Matrix to Eigen::Vector
    VectorXi vector_int = VectorXi::Map(matrix_int.data(), cells);
    
    // create vector to publish map
    int init_v[cells];
    std::vector<signed char> map_vector(init_v, init_v+cells);
    
    // convert vector of <int> to <int8_t> (signed char)
    for(int i=0; i<cells; i++) {
        map_vector[i] = (int8_t)vector_int[i];
    }
    
    // publish map
    map.data = map_vector;
    
    // check identifier and publish accordingly
    if(map.header.seq == staticMap_seq) {
        staticMapPub.publish(map);
    } else if(map.header.seq == staticMap_xxl_seq) {
        staticMapXxlPub.publish(map);
    } else if(map.header.seq == dynamicMap_seq) {
        dynamicMapPub.publish(map);
    } else if(map.header.seq == dynamicMap_xxl_seq) {
        dynamicMapXxlPub.publish(map);
    } else if(map.header.seq == inputMap_seq) {
        inputMapPub.publish(map);
    }
    
    if(debug)
    ROS_WARN("+++ Publishing map");    
    
}

// transform world coordinates to the matrix
void DynamicLayer::transformMapToMatrix(int map_x, int map_y, int &matrix_x, int &matrix_y)
{
        matrix_x = map_x - (master_grid_width/2 - width/2);
        matrix_y = map_y - (master_grid_height/2 - height/2);
}

void DynamicLayer::matchSize()
{ 
    Costmap2D* master = layered_costmap_->getCostmap();
    // resolution_ = layered_costmap->getResolution(); // Luke: Maybe usefull for a more generic approach... (as seen in inflation_layer)
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void DynamicLayer::reconfigureCB(scitos_2d_navigation::dynamic_layer_paramsConfig &config, uint32_t level)
{
    if(debug) ROS_INFO("Got params from dynamic reconfigure");
    
    // enable/disable whole layer   
    enabled_ = config.enabled;
    
    // enable/disable print debug output
    debug = config.debug_flag;
    
    // which maps to publish
    publish_fine_map = config.publish_fine_map;
    publish_block_map = config.publish_block_map;
    publish_input_map = config.publish_input_map;
    
    // get and recalculate parameters of algorithm
    stat_low = config.stat_low;
    stat_Low = stat_low/(1-stat_low);
    stat_low2 = config.stat_low2;
    stat_Low2 = stat_low2/(1-stat_low2);
    stat_high = config.stat_high;
    stat_High = stat_high/(1-stat_high);
    dyn_low = config.dyn_low;
    dyn_Low = dyn_low/(1-dyn_low);
    dyn_high = config.dyn_high;
    dyn_High = dyn_high/(1-dyn_high);
    
    // define publisher and shut them down if needed xxxPub.shutdown() ???
    if(publish_fine_map) {
        staticMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/move_base/" + name_ + "/static_map", 10);
        dynamicMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/move_base/" + name_ + "/dynamic_map", 10);
    }
    if(publish_block_map) {
        staticMapXxlPub = nh.advertise<nav_msgs::OccupancyGrid>("/move_base/" + name_ + "/static_map_xxl", 10);
        dynamicMapXxlPub = nh.advertise<nav_msgs::OccupancyGrid>("/move_base/" + name_ + "/dynamic_map_xxl", 10);
    }
    if(publish_input_map) {
        inputMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/move_base/" + name_ + "/input_map", 10);
    }
}

void DynamicLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
    return;

    if(debug) {
        ROS_INFO("-----------updateBounds--");
        ROS_INFO("origin_x:   %f", origin_x);
        ROS_INFO("origin_y:   %f", origin_y);
        ROS_INFO("origin_yaw: %f", origin_yaw);
        ROS_INFO("*min_x:     %f", *min_x);
        ROS_INFO("*min_y:     %f", *min_y); 
        ROS_INFO("*max_x:     %f", *max_x);
        ROS_INFO("*max_y:     %f", *max_y);
        ROS_INFO("-------------------------");
    }
}

void DynamicLayer::updateMaps(Eigen::MatrixXf &meas_mat, const int min_x, const int min_y, const int max_x, const int max_y, const bool xxl)
{
    /* 6 cases static update:
     * 
     * S_t-1        o_t        Diff    Equal     Avg        L/H
     * ========================================================
     * 0            0          0       True      0          L
     * 50           0          50      False     25         L
     * 100          0          100     False     50         L
     * 0            100        -100    False     50         L2  -> Problem for changing elements in environments
     * 50           100        -50     False     75         H
     * 100          100        0       True      100        H
     * 
     * 0            50         -50     False     25         enforce 0.5        (otherwise would be L)
     * 50           50         0       True      50         enforce 0.5        (otherwise would be H)
     * 100          50         50      False     75         enforce 0.5        (otherwise would be H)
     *  
     * 6 cases dynamic update:
     * S_t-1        o_t        Diff    Equal     Avg        L/H
     * ========================================================
     * 0            0          0       True      0          L
     * 50           0          50      False     25         L
     * 100          0          100     False     50         L
     * 0            100        -100    False     50         H
     * 50           100        -50     False     75         L
     * 100          100        0       True      100        L
     * 
     * 0            50         -50     False     25         L
     * 50           50         0       True      50         L
     * 100          50         50      False     75         L
     */
    
    if(debug) {
        ROS_WARN("+++ Updating static map");
        ROS_WARN_STREAM("(min_x, min_y): (" << min_x << ", " << min_y << ") -- (max_x, max_y): (" << max_x << ", " << max_y << ")");
    }
    
    if(xxl) {
        if(debug)
        ROS_INFO("Updating XXL maps...");
    } else {
        if(debug)
        ROS_INFO("Updating maps...");
    }
    
    // local variables
    float avg = 0;
    float diff = 0;
    float model = 0;
    float old_map = 0;
    
    float stat_map_value;
    float stat_map_value_upd;
    
    // buffer for filling up xxl map
    int i_xxl, j_xxl;
    
    for(int i=min_x; i<max_x; i++) {
        for(int j=min_y; j<max_y; j++) {
            // only look at actual measurements!
            if(meas_mat(i,j) != NO_INFORMATION) {
                
                // map input data to probabilistic representation
                if(meas_mat(i,j) == LETHAL_OBSTACLE) {
                    meas_mat(i,j) = 1;
                }
                
                // handle different input
                if(xxl) {
                    stat_map_value = staticMap_xxl_matrix(i,j);
                } else {
                    stat_map_value = staticMap_matrix(i,j);
                }
                
                //~ // calcluate average (for stat) to decide whether its L or H
                //~ avg = (meas_mat(i,j) + stat_map_value) / 2;
                // calcluate diff (for dyn) to decide whether its L or H
                diff = stat_map_value - meas_mat(i,j);
                
                // apply static model
                if(diff > 0) {
                    model = stat_Low;
                } else {
                    model = stat_High;
                    if(diff < -0.5) {        // second criteria for becoming L2
                        model = stat_Low2;
                    }
                }
                
                // never fully believe old static measurements
                old_map = (stat_map_value) / (1 - stat_map_value);
                            
                // finally calculate p( S^t | o^1, ... , o^t, S^(t-1) )
                if(meas_mat(i,j) == 0.5) {
                    if(xxl) {
                        staticMap_xxl_matrix(i,j) = 0.5;
                    } else {
                        staticMap_matrix(i,j) = 0.5;        // SUPER HACK!!! CHECK IF THAT IS A GOOD IDEA!!!
                    }
                } else {
                    if(xxl) {
                        staticMap_xxl_matrix(i,j) = std::min(map_max_value, std::max(map_min_value, (model*old_map) / (1 + model*old_map)));
                        stat_map_value_upd = staticMap_xxl_matrix(i,j);
                    } else {
                        staticMap_matrix(i,j) = std::min(map_max_value, std::max(map_min_value, (model*old_map) / (1 + model*old_map)));
                        stat_map_value_upd = staticMap_matrix(i,j);
                    }
                }
                
                // calcluate diff (for dyn) to decide whether its L or H (freshly... check that if necessary!) SUPER HACK
                //~ diff = stat_map_value_upd - meas_mat(i,j);
                
                // apply dynamic model
                if(diff < -upper_bound) {
                    model = dyn_High;
                } else {
                    model = dyn_Low;
                }
                
                if(xxl) {
                    // never fully believe old dynamic measurements
                    old_map = (dynamicMap_xxl_matrix(i,j)) / (1 - dynamicMap_xxl_matrix(i,j));
                    
                    // finally calculate p( D^t | o^1, ... , o^t, S^(t-1) )
                    dynamicMap_xxl_matrix(i,j) = std::min(map_max_value, std::max(map_min_value, (model*old_map) / (1 + model*old_map)));
                    //~ if(debug)
                        //~ ROS_INFO("Finished updating xxl maps");
                } else {
                    // never fully believe old dynamic measurements
                    old_map = (dynamicMap_matrix(i,j)) / (1 - dynamicMap_matrix(i,j));
                    
                    // finally calculate p( D^t | o^1, ... , o^t, S^(t-1) )
                    dynamicMap_matrix(i,j) = std::min(map_max_value, std::max(map_min_value, (model*old_map) / (1 + model*old_map)));
                }
                
                if(debug) {
                //~ std::cout << "|" << meas_mat(i,j) << ">" << old_dynamicMap_matrix_value << ">" << diff << ">" << model << ">" << old_map << ">" << dynamicMap_matrix(i,j) << " ";
                }
            }
        }
    }
}

void DynamicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    if (!enabled_)
    return;

    // first time the master_grid is available...
    if(!flag_init) {
        // get master_grid dimensions etc.
        master_grid_width = master_grid.getSizeInCellsX();
        master_grid_height = master_grid.getSizeInCellsY();
        master_grid_origin_x = master_grid.getOriginX();
        master_grid_origin_y = master_grid.getOriginY();
        master_grid_resolution = master_grid.getResolution();
        master_grid_orientation = 1.0;
        master_grid_width_m = master_grid.getSizeInMetersX();
        master_grid_height_m = master_grid.getSizeInMetersY();
        if(debug) {
            ROS_WARN_STREAM("+++ map dimensions of master_grid (px): " << master_grid_width << " x " << master_grid_height);
            ROS_WARN_STREAM("+++ map dimensions of master_grid (m): " << master_grid_width_m << " x " << master_grid_height_m);
            ROS_WARN_STREAM("+++ map origin [m]: (" << master_grid_origin_x << ", " << master_grid_origin_y << ")");
            ROS_WARN_STREAM("+++ map resolution: " << master_grid_resolution);
        }
        
        // initialize internal matrices
        //~ int width_meter_max = 60;
        //~ int height_meter_max = 30;
        max_cells_xxl_x = (int)(max_cells_x * master_grid_resolution/resolution_xxl +0.5);
        max_cells_xxl_y = (int)(max_cells_y * master_grid_resolution/resolution_xxl +0.5);
        //~ double width_double  = width_meter_max   / master_grid_resolution;
        //~ double height_double = height_meter_max  / master_grid_resolution;
        width  = std::min(max_cells_x, master_grid_width); //(int)(width_double+0.5);
        height = std::min(max_cells_y, master_grid_height); //(int)(height_double+0.5);
        width_xxl   = std::min(max_cells_xxl_x, (int)(master_grid.getSizeInMetersX() / resolution_xxl + 0.5)); //width_meter_max   / resolution_xxl;
        height_xxl  = std::min(max_cells_xxl_y, (int)(master_grid.getSizeInMetersY() / resolution_xxl + 0.5));

        n_cells = width*height;
        n_cells_xxl = width_xxl*height_xxl;
        
        if(debug)
            ROS_INFO_STREAM("width_xxl: " << width_xxl << " -- height_xxl: " << height_xxl << " -- width_xxl*height_xxl: " << width_xxl*height_xxl);
        
        mod_number = (int)(resolution_xxl / master_grid_resolution +0.5);            // good idea to introduce an exception handler here
                
        min_i = std::max(min_i, master_grid_width/2  - width/2);
        min_j = std::max(min_j, master_grid_height/2 - height/2);
        max_i = std::min(max_i, master_grid_width/2  + width/2);
        max_j = std::min(max_j, master_grid_height/2 + height/2);
    }
    
    // handle all init here
    if(publish_fine_map && !flag_init_fine) {
        initStaticMap();
        initDynamicMap();
        ROS_INFO_STREAM("Initialized fine maps with " << width << " X " << height);
        staticMap_matrix = MatrixXf::Constant(width, height, 0.5);
        dynamicMap_matrix = MatrixXf::Constant(width, height, 0.5);
        // do not set flag_init_fine to true because we may want to initialize it with the map
    }
    if(publish_block_map && !flag_init_block) {
        initStaticMapXxl();
        initDynamicMapXxl();
        staticMap_xxl_matrix = MatrixXf::Constant(width_xxl, height_xxl, 0.5);
        dynamicMap_xxl_matrix = MatrixXf::Constant(width_xxl, height_xxl, 0.5);
        flag_init_block = true;
    }
    if(publish_input_map && !flag_init_input) {
        initInputMap();
        flag_init_input = true;
    }
        
    if(debug) {
        ROS_INFO("------------updateCosts--");
        ROS_INFO("min_i:      %i", min_i);
        ROS_INFO("min_j:      %i", min_j); 
        ROS_INFO("max_i:      %i", max_i);
        ROS_INFO("max_j:      %i", max_j);
        ROS_INFO("-------------------------");
        ROS_INFO_STREAM("width, height: " << width << "x" << height << " -- width_xxl, height_xxl: " << width_xxl << "x" << height_xxl);
    }
    
    
    // initialize input data matrix (only in scope to protect from misbehaving...)
    MatrixXf inputData_matrix = MatrixXf::Constant(width,height,-1);
    MatrixXf inputData_xxl_matrix = MatrixXf::Constant(width_xxl,height_xxl,-1);
        
    //~ costmap_2d::Costmap2D* layered_costmap = layered_costmap_->getCostmap();
    //~ unsigned int size_x = layered_costmap->getSizeInCellsX(), size_y = layered_costmap->getSizeInCellsY();
    //~ unsigned char* master_array = layered_costmap->getCharMap();
//~ 
    //~ int counter_ma = 0;
    //~ for (int j = min_j; j < max_j; j++) {
        //~ for (int i = min_i; i < max_i; i++) {
            //~ int index_ma = layered_costmap->getIndex(i, j);
            //~ counter_ma++;
        //~ }
    //~ }
    
    if(debug) ROS_INFO_STREAM("Modnumber: " << mod_number << " ");

    int matrix_x, matrix_y, i_xxl, j_xxl, mat_x_xxl, mat_y_xxl;
    //~ int counter = 0;
    bool obstacle_present;
    bool unknown_present;
    float local_sum;
    //~ for (int j = min_j; j < max_j; j++)
    //~ {
        //~ for (int i = min_i; i < max_i; i++)
        //~ {
            //~ counter++;
            //~ int index = layered_costmap->getIndex(i, j);
            //~ transformMapToMatrix(i, j, matrix_x, matrix_y);
            //~ if(flag_init) {
                //~ inputData_matrix(matrix_x,matrix_y) = (int)master_array[index];
                //~ 
                //~ // sample the inputData matrix algorithm
                //~ if(i%mod_number == (mod_number-1) && j%mod_number == (mod_number-1)) {            // last element
                    //~ transformMapToMatrix(i, j, mat_x_xxl, mat_y_xxl);
                    //~ i_xxl = mat_x_xxl/mod_number; j_xxl = mat_y_xxl/mod_number;
                    //~ 
                    //~ if(debug)
                    //~ ROS_WARN_STREAM("(i_xxl, j_xxl) = (" << i_xxl << ", " << j_xxl << ")");
                    //~ 
                    //~ // find the max of the lower resolution grid
                    //~ obstacle_present = false;
                    //~ unknown_present = false;
                    //~ local_sum = 0;
                    //~ for(int i_tmp=0; i_tmp<mod_number; i_tmp++) {
                        //~ for(int j_tmp=0; j_tmp<mod_number; j_tmp++) {
                            //~ // overrules everything
                            //~ if(inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp) == LETHAL_OBSTACLE) {
                                //~ obstacle_present = true;
                            //~ }
                            //~ // second priority
                            //~ if(inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp) == NO_INFORMATION) {
                                //~ unknown_present = true;
                            //~ }
                            //~ // else
                            //~ local_sum += inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp);
                        //~ }
                    //~ }
                    //~ 
                    //~ if(obstacle_present) {
                        //~ inputData_xxl_matrix(i_xxl,j_xxl) = LETHAL_OBSTACLE;
                    //~ } else if(unknown_present) {
                        //~ inputData_xxl_matrix(i_xxl,j_xxl) = NO_INFORMATION;
                    //~ } else {
                        //~ inputData_xxl_matrix(i_xxl,j_xxl) = local_sum/(mod_number*mod_number);
                    //~ }
                //~ }            
            //~ }
            //~ 
//~ 
        //~ }
    //~ }
    
      
    // search through all layers of the layered_costmap_
    std::string search_string_1 = "smart_obstacle";
    std::string search_string_2 = "obstacle";
    
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = layered_costmap_->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        if((plugin->getName().find(search_string_1) != std::string::npos) || (plugin->getName().find(search_string_2) != std::string::npos)) {
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            unsigned char* grid = costmap->getCharMap();
            
            if(debug) ROS_INFO_STREAM("Got costmap - w x h: " << width << " x " << height);
            
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {                    
                
                    int index = costmap->getIndex(i, j);
                    transformMapToMatrix(i, j, matrix_x, matrix_y);
                    
                    if(flag_init) {
                                                
                        inputData_matrix(matrix_x,matrix_y) = (int)grid[index];
                        
                        // sample the inputData matrix algorithm
                        if(i%mod_number == (mod_number-1) && j%mod_number == (mod_number-1)) {            // last element
                            transformMapToMatrix(i, j, mat_x_xxl, mat_y_xxl);
                            i_xxl = mat_x_xxl/mod_number; j_xxl = mat_y_xxl/mod_number;
                            
                            //~ if(debug)
                            //~ ROS_WARN_STREAM("(i_xxl, j_xxl) = (" << i_xxl << ", " << j_xxl << ")");
                            
                            // find the max of the lower resolution grid
                            obstacle_present = false;
                            unknown_present = false;
                            local_sum = 0;
                            for(int i_tmp=0; i_tmp<mod_number; i_tmp++) {
                                for(int j_tmp=0; j_tmp<mod_number; j_tmp++) {
                                    // overrules everything
                                    if(inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp) == LETHAL_OBSTACLE) {
                                        obstacle_present = true;
                                    }
                                    // second priority
                                    if(inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp) == NO_INFORMATION) {
                                        unknown_present = true;
                                    }
                                    // else
                                    local_sum += inputData_matrix(mat_x_xxl-i_tmp,mat_y_xxl-j_tmp);
                                }
                            }
                            
                            if(obstacle_present) {
                                inputData_xxl_matrix(i_xxl,j_xxl) = LETHAL_OBSTACLE;
                            } else if(unknown_present) {
                                inputData_xxl_matrix(i_xxl,j_xxl) = NO_INFORMATION;
                            } else {
                                inputData_xxl_matrix(i_xxl,j_xxl) = local_sum/(mod_number*mod_number);
                            }
                        }            
                    }
                }
            }
                        
            // which one is which -> color cell (2,3) -> works (index starts with (0,0)
            if(debug) {
                int index = costmap->getIndex(2, 3); // (i, j)
                transformMapToMatrix(2, 3, matrix_x, matrix_y);
                if(flag_init) {
                    inputData_matrix(matrix_x, matrix_y) = 1.0;
                }
            }
            
        }
    }      
    
        
    if(flag_init) {
        if(debug) ROS_INFO("I handled first input correctly and also initalized successfully.");
        // mark end points
        if(debug && publish_fine_map) {
            transformMapToMatrix(max_i, min_j, matrix_x, matrix_y);
            staticMap_matrix(matrix_x, matrix_y) = 150;
            transformMapToMatrix(min_i, max_j, matrix_x, matrix_y);
            staticMap_matrix(matrix_x, matrix_y) = 150;
            transformMapToMatrix(max_i, max_j, matrix_x, matrix_y);
            staticMap_matrix(matrix_x, matrix_y) = 150;
            transformMapToMatrix(min_i, min_j, matrix_x, matrix_y);
            staticMap_matrix(matrix_x, matrix_y) = 150;
        }
        
        // transform min, max
        int min_x, min_y, max_x, max_y;
        transformMapToMatrix(min_i, min_j, min_x, min_y);
        transformMapToMatrix(max_i, max_j, max_x, max_y);
        
        // update static and dynamic maps
        if(publish_fine_map) {
            updateMaps(inputData_matrix, min_x, min_y, max_x, max_y, false);
        }
        
        // update static and dynamic xxl maps
        if(publish_block_map) {
            if(debug) {
                ROS_INFO("Before publishing xxl map");
            }
            int min_x_xxl, min_y_xxl, max_x_xxl, max_y_xxl;
            min_x_xxl = min_x/mod_number;
            min_y_xxl = min_y/mod_number;
            max_x_xxl = max_x/mod_number;
            max_y_xxl = max_y/mod_number;
            updateMaps(inputData_xxl_matrix, min_x_xxl, min_y_xxl, max_x_xxl, max_y_xxl, true);
        }
        
        // publish Maps
        if(publish_fine_map) {
            if(debug) ROS_WARN("Right before publishing");
            publishMap(staticMap, staticMap_matrix, n_cells);
            publishMap(dynamicMap, dynamicMap_matrix, n_cells);
            if(debug) ROS_WARN("Right after publishing");
        }
        if(publish_block_map) {
            publishMap(staticMap_xxl, staticMap_xxl_matrix, n_cells_xxl);
            publishMap(dynamicMap_xxl, dynamicMap_xxl_matrix, n_cells_xxl);
        }
        if(publish_input_map) {
            publishMap(inputMap, inputData_matrix, n_cells);
        }
    }
    
    // initialize staticMap with the values of the static_layer
    
    costmap_2d::Costmap2D* layered_costmap = layered_costmap_->getCostmap();
    unsigned char* init_map_array = layered_costmap->getCharMap();
    
    if(publish_fine_map && !flag_init_fine) {
        for(int i=min_i; i<max_i; i++) {
            for(int j=min_j; j<max_j; j++) {
                int index = layered_costmap->getIndex(i, j);
                transformMapToMatrix(i, j, matrix_x, matrix_y);
                //~ ROS_ERROR_STREAM("I was able to transform " << i << " " << j << " " << matrix_x << " " << matrix_y);
                if((int)init_map_array[index] == NO_INFORMATION) {
                    staticMap_matrix(matrix_x, matrix_y) = 0.5;
                    continue;
                } else if((int)init_map_array[index] == FREE_SPACE) {
                    if(init_fine_blank) {
                        staticMap_matrix(matrix_x, matrix_y) = 0.5;
                    } else {
                        staticMap_matrix(matrix_x, matrix_y) = map_min_value;
                    }
                    continue;
                } else if((int)init_map_array[index] == LETHAL_OBSTACLE) {
                    if(init_fine_blank) {
                        staticMap_matrix(matrix_x, matrix_y) = 0.5;
                    } else {
                        staticMap_matrix(matrix_x, matrix_y) = map_max_value;
                    }
                    continue;
                } else {
                    ROS_WARN("Cannot introduce ano unknown value to the map...");
                }
            }
        }
        flag_init_fine = true;
    }
        
    // only do that once
    flag_init = true;

}

} // end namespace
