
#ifndef LASER_MERGER2_NODE_HPP_
#define LASER_MERGER2_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"   
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "laser_merger2/visibility_control.h"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

typedef struct{
	float x;
  float y;
  float z; 
  std::optional<float> intensity; // Optional intensity field
} SCAN_POINT_t;

class laser_merger2 : public rclcpp::Node
{
  public:
    laser_merger2();
    ~laser_merger2();

  private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<SCAN_POINT_t> scantoPointXYZ(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    Eigen::Matrix4d Rotate3Z(double rad);
    Eigen::Matrix4d ConvertTransMatrix(geometry_msgs::msg::TransformStamped trans);
    uint32_t rgb_to_uint32(uint8_t r, uint8_t g, uint8_t b);
    void ConvertPointCloud2(std::vector<SCAN_POINT_t> points);
    void ConvertLaserScan(std::vector<SCAN_POINT_t> points);
    void laser_merge();

    std::mutex nodeMutex_;

    std::unique_ptr<tf2_ros::Buffer> tf2_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_sub;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pclPub_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scanPub_;

    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> scanBuffer;

    std::thread subscription_listener_thread_;
    std::atomic_bool alive_{true};

    laser_geometry::LaserProjection projector_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time laserTime;

    // ROS Parameters
    std::shared_ptr<rclcpp::Rate> rosRate;
    std::string target_frame_;
    std::vector<std::string> scan_topics;
    std::vector<std::string> qos_profiles;
    double tolerance_;
    double rate_;
    int input_queue_size_;
    int subscription_count;

    double max_range;
    double min_range;
    double max_angle;
    double min_angle;
    double scan_time;
    double angle_increment;
    double inf_epsilon;
    bool use_inf;
};

#endif
