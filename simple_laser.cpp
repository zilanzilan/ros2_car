#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <string>

class LidarAngleMonitor : public rclcpp::Node
{
public:
    LidarAngleMonitor()
    : Node("lidar_angle_monitor")
    {
        // 声明参数：目标角度（度数），默认 10.0
        this->declare_parameter<double>("target_angle_deg", 10.0);
        target_angle_deg_ = this->get_parameter("target_angle_deg").as_double();

        // 订阅激光雷达话题 /scan
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,
            std::bind(&LidarAngleMonitor::scan_callback, this, std::placeholders::_1));

        // 创建定时器：每秒打印一次
        timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&LidarAngleMonitor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Lidar Angle Monitor Started. Listening for %.1f°", target_angle_deg_);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. 将目标角度从度数转换为弧度
        double target_angle_rad = target_angle_deg_ * M_PI / 180.0;

        // 2. 计算最近邻索引
        float float_index = (target_angle_rad - msg->angle_min) / msg->angle_increment;
        int nearest_index = static_cast<int>(std::round(float_index));

        // 3. 边界检查
        if (nearest_index >= 0 && nearest_index < static_cast<int>(msg->ranges.size())) {
            float distance = msg->ranges[nearest_index];

            // 检查数据有效性 (非 inf, 非 NaN, 且在量程内)
            if (std::isfinite(distance) && distance >= msg->range_min && distance <= msg->range_max) {
                latest_distance_ = distance;
                has_valid_data_ = true;
            } else {
                // 数据无效（例如无回波返回 inf）
                has_valid_data_ = false;
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Calculated index %d is out of bounds [0, %zu]", 
                                 nearest_index, msg->ranges.size());
            has_valid_data_ = false;
        }
    }

    void timer_callback()
    {
        if (has_valid_data_) {
            RCLCPP_INFO(this->get_logger(), 
                        "Target: %.1f° | Distance: %.3f m", 
                        target_angle_deg_, 
                        latest_distance_);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                        "Target: %.1f° | Distance: No valid data (Out of range or Inf/NaN)", 
                        target_angle_deg_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    double target_angle_deg_;
    float latest_distance_;
    bool has_valid_data_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarAngleMonitor>());
    rclcpp::shutdown();
    return 0;
}