#ifndef SMART_OBSTACLE_COSTMAP_PLUGIN_H_
#define SMART_OBSTACLE_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint_layer.h>

#include <scitos_2d_navigation/smart_obstacle_layer_paramsConfig.h>


namespace scitos_2d_navigation
{
class SmartObstacleLayer : public costmap_2d::CostmapLayer
{
public:
    SmartObstacleLayer()
    {
        costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
    }

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                                         double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    /**
* @brief A callback to handle buffering LaserScan messages
* @param message The message returned from a message notifier
* @param buffer A pointer to the observation buffer to update
*/
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                                 const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

     /**
* @brief A callback to handle buffering LaserScan messages which need to be filtered to turn Inf values into range_max.
* @param message The message returned from a message notifier
* @param buffer A pointer to the observation buffer to update
*/
     void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                                                                    const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

    /**
* @brief A callback to handle buffering PointCloud messages
* @param message The message returned from a message notifier
* @param buffer A pointer to the observation buffer to update
*/
    void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                                    const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

    /**
* @brief A callback to handle buffering PointCloud2 messages
* @param message The message returned from a message notifier
* @param buffer A pointer to the observation buffer to update
*/
    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                     const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

    void setResetBounds(double mx0, double mx1, double my0, double my1)
    {
        reset_min_x_ = std::min(mx0, reset_min_x_);
        reset_max_x_ = std::max(mx1, reset_max_x_);
        reset_min_y_ = std::min(my0, reset_min_y_);
        reset_max_y_ = std::max(my1, reset_max_y_);
        has_been_reset_ = true;
    }

    // for testing purposes
    void addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing);

protected:

    virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

    /**
* @brief Get the observations used to mark space
* @param marking_observations A reference to a vector that will be populated with the observations
* @return True if all the observation buffers are current, false otherwise
*/
    bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

    /**
* @brief Get the observations used to clear space
* @param clearing_observations A reference to a vector that will be populated with the observations
* @return True if all the observation buffers are current, false otherwise
*/
    bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;

    /**
* @brief Clear freespace based on one observation
* @param clearing_observation The observation used to raytrace
* @param min_x
* @param min_y
* @param max_x
* @param max_y
*/
    virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                                                 double* max_x, double* max_y);

    void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
double* max_x, double* max_y);

    /** @brief Overridden from superclass Layer to pass new footprint into footprint_layer_. */
    virtual void onFootprintChanged();

    std::string global_frame_; ///< @brief The global frame for the costmap
    double max_obstacle_height_; ///< @brief Max Obstacle Height

    laser_geometry::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds

    std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_; ///< @brief Used for the observation message filters
    std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
    std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_; ///< @brief Used to store observations from various sensors
    std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_; ///< @brief Used to store observation buffers used for marking obstacles
    std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_; ///< @brief Used to store observation buffers used for clearing obstacles

    // Used only for testing purposes
    std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;

    bool rolling_window_;
    dynamic_reconfigure::Server<scitos_2d_navigation::smart_obstacle_layer_paramsConfig> *dsrv_;

    bool has_been_reset_;
    double reset_min_x_, reset_max_x_, reset_min_y_, reset_max_y_;

    costmap_2d::FootprintLayer footprint_layer_; ///< @brief clears the footprint in this obstacle layer.
    
    int combination_method_;

private:
    void reconfigureCB(scitos_2d_navigation::smart_obstacle_layer_paramsConfig &config, uint32_t level);
    
    // smart part
    bool only_current_measurements_;

};
}
#endif