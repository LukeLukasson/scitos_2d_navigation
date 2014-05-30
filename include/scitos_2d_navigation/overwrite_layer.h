#ifndef OVERWRITE_COSTMAP_PLUGIN_H_
#define OVERWRITE_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
//~ #include <costmap_2d/layer.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <scitos_2d_navigation/overwrite_layer_paramsConfig.h>

namespace scitos_2d_navigation
{
class OverwriteLayer : public costmap_2d::CostmapLayer //public costmap_2d::Layer, public costmap_2d::Costmap2D, 
{
public:
    OverwriteLayer();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void matchSize();

private:
  /**
* @brief Callback to update the costmap's map from the map_server
* @param new_map The map to put into the costmap. The origin of the new
* map along with its size will determine what parts of the costmap's
* static map are overwritten.
*/
    void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
    void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
    void reconfigureCB(scitos_2d_navigation::overwrite_layer_paramsConfig &config, uint32_t level);

    unsigned char interpretValue(unsigned char value);

    std::string global_frame_; ///< @brief The global frame for the costmap
    bool subscribe_to_updates_;
    bool map_received_;
    bool has_updated_data_;
    unsigned int x_,y_,width_,height_;
    bool track_unknown_space_;
    bool use_maximum_;
    bool trinary_costmap_;
    ros::Subscriber map_sub_, map_update_sub_;

    unsigned char lethal_threshold_, unknown_cost_value_;

    mutable boost::recursive_mutex lock_;
    dynamic_reconfigure::Server<scitos_2d_navigation::overwrite_layer_paramsConfig> *dsrv_;
};
}
#endif
