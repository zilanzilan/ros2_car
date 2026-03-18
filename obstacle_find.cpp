#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

class LaserObstacleDetector : public rclcpp::Node
{
public:
    LaserObstacleDetector() : Node("laser_obstacle_detector")
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("robot_namespace", "ack11");
        this->declare_parameter<float>("obstacle_threshold", 2.0f);
        this->declare_parameter<int>("max_gap", 10);

        robot_namespace_ = this->get_parameter("robot_namespace").as_string();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        max_gap_ = this->get_parameter("max_gap").as_int();

        // 构建话题名称
        std::string scan_topic = "/" + robot_namespace_ + "/scan";
        std::string center_topic = "/" + robot_namespace_ + "/obstacle_centers";

        // 创建订阅者（使用传感器数据默认QoS）
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaserObstacleDetector::scanCallback, this, std::placeholders::_1));

        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(center_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Laser obstacle detector initialized for namespace: %s", robot_namespace_.c_str());
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        processObstacles(scan_msg);
    }

    void processObstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        const std::vector<float>& ranges = scan_msg->ranges;
        size_t N = ranges.size();
        if (N == 0) return;

        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;

        std::vector<bool> is_obstacle(N, false);

        // Step 1: 标记障碍物点
        for (size_t i = 0; i < N; ++i) {
            float r = ranges[i];
            if (!std::isfinite(r)) continue;
            if (r < obstacle_threshold_) {
                is_obstacle[i] = true;
            }
        }

        // Step 2: 聚类障碍物（考虑环形结构）
        std::vector<std::vector<size_t>> clusters;
        std::vector<bool> visited(N, false);

        for (size_t start = 0; start < N; ++start) {
            if (!is_obstacle[start] || visited[start]) continue;

            std::vector<size_t> cluster;
            size_t current = start;

            while (!visited[current]) {
                if (!is_obstacle[current]) break;
                visited[current] = true;
                cluster.push_back(current);

                size_t next = (current + 1) % N;
                int gap = 0;
                while (gap < max_gap_ && !is_obstacle[next]) {
                    gap++;
                    next = (next + 1) % N;
                    if (next == start) break;
                }

                if (gap < max_gap_ && is_obstacle[next] && !visited[next]) {
                    current = next;
                } else {
                    break;
                }
            }

            clusters.push_back(cluster);
        }

        // 合并首尾聚类（如果它们在环上相邻）
        if (clusters.size() >= 2) {
            const auto& first_cluster = clusters.front();
            const auto& last_cluster = clusters.back();

            size_t last_max = *std::max_element(last_cluster.begin(), last_cluster.end());
            size_t first_min = *std::min_element(first_cluster.begin(), first_cluster.end());

            if ((N - 1 - last_max) + first_min < static_cast<size_t>(max_gap_)) {
                std::vector<size_t> merged = last_cluster;
                merged.insert(merged.end(), first_cluster.begin(), first_cluster.end());
                clusters[0] = merged;
                clusters.pop_back();
            }
        }

        // 提取每个聚类的中心点并发布
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        cloud_msg->header = scan_msg->header;  // 使用与激光扫描相同的时间戳和坐标系
        cloud_msg->header.stamp = this->now(); // 也可以使用当前时间，但保持原样更合适

        for (auto& cluster : clusters) {
            if (cluster.empty()) continue;

            size_t mid_index = cluster[cluster.size() / 2];
            float r = ranges[mid_index];
            if (!std::isfinite(r)) continue;

            float angle = angle_min + static_cast<float>(mid_index) * angle_increment;
            float x = r * std::cos(angle);
            float y = r * std::sin(angle);

            geometry_msgs::msg::Point32 pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.0f;
            cloud_msg->points.push_back(pt);
        }

        publisher_->publish(std::move(cloud_msg));
    }

    std::string robot_namespace_;
    double obstacle_threshold_;
    int max_gap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}