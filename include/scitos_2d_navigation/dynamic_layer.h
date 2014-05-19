#ifndef DYNAMIC_LAYER_H_
#define DYNAMIC_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

// Luke
#include <iostream>
#include "../Eigen/Dense" // adjusted path
#include <nav_msgs/OccupancyGrid.h>


namespace scitos_2d_navigation
{
    
class DynamicLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    // Destructor 
    ~DynamicLayer();

    // from original layer structure
    virtual void onInitialize();
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }

    virtual void matchSize();
  
private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    // Luke
    // worker functions
    void publishMap(nav_msgs::OccupancyGrid &map, Eigen::MatrixXf &matrix, int cells);
    void initStaticMap();
    void initStaticMapXxl();
    void initDynamicMap();
    void initDynamicMapXxl();
    void transformMapToMatrix(int world_x, int world_y, int &map_x, int &map_y);
    void updateMaps(Eigen::MatrixXf &meas_mat, const int min_x, const int min_y, const int max_x, const int max_y, const bool xxl);

    // make the matrix available
    Eigen::MatrixXf staticMap_matrix;
    Eigen::MatrixXf staticMap_xxl_matrix;
    Eigen::MatrixXf dynamicMap_matrix;
    Eigen::MatrixXf dynamicMap_xxl_matrix;
    
    // maps
    nav_msgs::OccupancyGrid staticMap;
    nav_msgs::OccupancyGrid staticMap_xxl;
    nav_msgs::OccupancyGrid dynamicMap;
    nav_msgs::OccupancyGrid dynamicMap_xxl;
    
    // identifier
    int staticMap_seq;
    int staticMap_xxl_seq;
    int dynamicMap_seq;
    int dynamicMap_xxl_seq;
    
    // ROS handles
    ros::NodeHandle nh;
    ros::Publisher staticMapPub;
    ros::Publisher staticMapXxlPub;
    ros::Publisher dynamicMapPub;
    ros::Publisher dynamicMapXxlPub;

    // grid data
    int height;
    int width;
    int height_xxl;
    int width_xxl;
    double resolution;
    double resolution_xxl;
    int n_cells;
    int n_cells_xxl;
    int max_cells_x;
    int max_cells_y;
    int max_cells_xxl_x;
    int max_cells_xxl_y;
    int mod_number;
    int master_grid_width;
    int master_grid_height;
    double master_grid_origin_x;
    double master_grid_origin_y;
    double master_grid_orientation;
    double master_grid_resolution;
    
    // flags
    bool flag_init;
    bool debug;
    bool init_fine_blank;
    // debugging flags
    bool publish_fine_map;
    bool publish_block_map;

    // parameters algorithm
    float lower_bound;
    float upper_bound;
    
    float map_min_value;
    float map_max_value;
    
    double stat_High;
    double stat_Low;
    double stat_Low2;
    double dyn_High;
    double dyn_Low;
};
}
#endif
