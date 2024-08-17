#include <laser_merger2/laser_merger2.h>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <boost/bind.hpp>

laser_merger2::laser_merger2() : Node("laser_merger2")
{
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<double>("transform_tolerance", 0.01);
    this->declare_parameter<double>("rate", 30.0);
    this->declare_parameter<int>("queue_size", 20);

    this->declare_parameter<double>("max_range", 30.0);
    this->declare_parameter<double>("min_range", 0.06);
    this->declare_parameter<double>("max_angle", 3.141592654);
    this->declare_parameter<double>("min_angle", -3.141592654);
    this->declare_parameter<double>("scan_time", 1.0 / 30.0);
    this->declare_parameter<double>("angle_increment", M_PI / 180.0);
    this->declare_parameter<double>("inf_epsilon", 1.0);
    this->declare_parameter<bool>("use_inf", true);

    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("transform_tolerance", tolerance_);
    this->get_parameter("rate", rate_);
    this->get_parameter("queue_size", input_queue_size_);

    this->get_parameter("max_range", max_range);
    this->get_parameter("min_range", min_range);
    this->get_parameter("max_angle", max_angle);
    this->get_parameter("min_angle", min_angle);
    this->get_parameter("scan_time", scan_time);
    this->get_parameter("angle_increment", angle_increment);
    this->get_parameter("inf_epsilon", inf_epsilon);
    this->get_parameter("use_inf", use_inf);

    pclPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", input_queue_size_);
    scanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", input_queue_size_);

    rosRate = std::make_shared<rclcpp::Rate>(rate_);

    if (!target_frame_.empty())
    {
        tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

        laser_sub.resize(USE_LASER_NUM);
        tfInit.resize(USE_LASER_NUM);
        for(int i = 0; i < USE_LASER_NUM; i++)
        {
            std::string ScanTopicName;
            ScanTopicName = "/scan_" + std::to_string(i);
            laser_sub[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(ScanTopicName, 10, [this, i,  ScanTopicName](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                scanCallback(msg, i);
            }
            );
        }
    }
    
    subscription_listener_thread_ = std::thread(std::bind(&laser_merger2::laser_merge, this));
}

laser_merger2::~laser_merger2()
{
    subscription_listener_thread_.join();
    alive_.store(true);
}


void laser_merger2::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, int laserIdx)
{
    std::lock_guard<std::mutex> lock(nodeMutex_);

    laserTime = scan->header.stamp;
    if(!tfInit[laserIdx])
    {
        scanData[laserIdx].dataSize = scan->ranges.size();
        InitDataBuffer(laserIdx, scanData[laserIdx].dataSize);
        CalculateMatrix(laserIdx, scan->header.frame_id);
        tfInit[laserIdx] = true;
    }
    else
    {
        memcpy(&scanData[laserIdx].pose[0], scan->ranges.data(), scanData[laserIdx].dataSize * sizeof(float));
        ScanToTransformedPointCloud(scanData[laserIdx].pose, scan->angle_min, scan->angle_max, scanData[laserIdx].dataSize, scanData[laserIdx].tranMatrixTF, scanData[laserIdx].outPose);
    }
}

void laser_merger2::InitDataBuffer(int laserIdx, int dataNum)
{
    float *unitPtr;
    InitGpuBuffer(&scanData[laserIdx].pose, &scanData[laserIdx].tranMatrixTF, &scanData[laserIdx].outPose, dataNum);
    unitPtr = &scanData[laserIdx].pose[dataNum * 3];

    for(int i = 0 ; i < dataNum ; i ++)
    {
        *unitPtr = 1;
        unitPtr++;
    }
}

Eigen::Matrix4f laser_merger2::ConvertTransMatrix(geometry_msgs::msg::TransformStamped trans)
{
    Eigen::Matrix4f res;
    // Convert geometry_msgs quaternion to tf2 quaternion
    tf2::Quaternion quaternion;
    tf2::fromMsg(trans.transform.rotation, quaternion);

    // Extract RPY angles from tf2 quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

	res.setZero();
    res(0, 0) = cos(yaw);
    res(0, 1) = -sin(yaw);
    res(1, 0) = sin(yaw);
    res(1, 1) = cos(yaw);
	res(0, 3) = trans.transform.translation.x;
	res(1, 3) = trans.transform.translation.y;
	res(2, 3) = trans.transform.translation.z;
	res(2, 2) = 1.0f;
	res(3, 3) = 1.0f;

	return res;
}

void laser_merger2::CalculateMatrix(int laserIdx, std::string frame)
{
    geometry_msgs::msg::TransformStamped sensorToBase;

    try
    {
        sensorToBase = tf2_->lookupTransform(target_frame_, frame, tf2::TimePointZero);
    }
    catch(const tf2::TransformException & ex)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame_.c_str(), frame.c_str(), ex.what());
        return;
    }

    const Eigen::Matrix4f T = ConvertTransMatrix(sensorToBase);
    memcpy(&scanData[laserIdx].tranMatrixTF[0], T.data(), 4 * 4 * sizeof(float));
}

uint32_t laser_merger2::rgb_to_uint32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

void laser_merger2::ConvertPointCloud2(std::vector<SCAN_POINT_t> points)
{
    auto pclMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    
    pclMsg->header.frame_id = target_frame_;
    pclMsg->header.stamp = laserTime;

    pclMsg->height = 1;
    pclMsg->width = points.size();

    sensor_msgs::PointCloud2Modifier modifier(*pclMsg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    /*modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                     "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);*/

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pclMsg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pclMsg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pclMsg, "z");
    //sensor_msgs::PointCloud2Iterator<float> iter_rgb(*pclMsg, "rgb");

    for(size_t i = 0; i < pclMsg->width; i++)
    {
        //uint32_t rgb_value = rgb_to_uint32(255, 0, 0);
        *iter_x = points[i].x;
        *iter_y = points[i].y;
        *iter_z = points[i].z;

        /*float* rgb_ptr = reinterpret_cast<float*>(&rgb_value);
        *iter_rgb = *rgb_ptr;*/

        ++iter_x; ++iter_y; ++iter_z; //++iter_rgb;
    }

    pclPub_->publish(*pclMsg);
}

void laser_merger2::ConvertLaserScan(std::vector<SCAN_POINT_t> points)
{
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp = laserTime;
    scan_msg->header.frame_id = target_frame_;
    
    scan_msg->angle_min = min_angle;
    scan_msg->angle_max = max_angle;
    scan_msg->angle_increment = angle_increment;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time;
    scan_msg->range_min = min_range;
    scan_msg->range_max = max_range;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    
    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if(use_inf)
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    else
        scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon);
    
    for(size_t i = 0; i < points.size(); i++)
    {
        double range = hypot(points[i].x, points[i].y);
        double angle = atan2(points[i].y, points[i].x);
        if(range < min_range || range > max_range || angle < scan_msg->angle_min || angle > scan_msg->angle_max)
        {
            continue;
        }
        
        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
        if(range < scan_msg->ranges[index])
        {
            scan_msg->ranges[index] = range;
        }
    }

    scanPub_->publish(std::move(scan_msg));
}

void laser_merger2::laser_merge()
{
    rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
    
    while(rclcpp::ok(context) && alive_.load())
    {
        std::vector<SCAN_POINT_t> points;
        
        // convert all scans to current base frame
        for(int i = 0; i < USE_LASER_NUM; i++)
        {
            float *iter_x = scanData[i].outPose;
            float *iter_y = scanData[i].outPose + scanData[i].dataSize - 1;
            float *iter_z = scanData[i].outPose + scanData[i].dataSize * 2 - 1;

            for(int j = 0; j < scanData[i].dataSize; j++)
            {
                SCAN_POINT_t pose;
                pose.x = *iter_x;
                pose.y = *iter_y;
                pose.z = *iter_z;
                //printf("x = %f, y = %f, z = %f\n", pose.x, pose.y, pose.z);

                points.push_back(pose);
                iter_x ++; iter_y ++; iter_z ++;
            }
        }

        {
            std::lock_guard<std::mutex> lock(nodeMutex_);
            ConvertPointCloud2(points);
            ConvertLaserScan(points);
        }

        rosRate->sleep();
    }
    
}