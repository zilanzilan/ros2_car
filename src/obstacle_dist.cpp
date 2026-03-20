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
        this->declare_parameter<float>("cluster_distance_threshold", 0.09f);  // 聚类距离阈值(平方距离)------------------------------------------------------

        robot_namespace_ = this->get_parameter("robot_namespace").as_string();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();

        // 构建话题名称
        std::string scan_topic = "/scan";
        std::string center_topic = "/obstacle_centers";

        // 创建订阅者（使用传感器数据默认QoS）
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaserObstacleDetector::scanCallback, this, std::placeholders::_1));

        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(center_topic, 10);
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

        // Step 1: 收集所有障碍物点（距离小于阈值且有限）的索引和坐标
        std::vector<size_t> indices;
        std::vector<geometry_msgs::msg::Point32> points;

        for (size_t i = 0; i < N; ++i) {
            float r = ranges[i];
            if (!std::isfinite(r) || r >= obstacle_threshold_) continue;

            float angle = angle_min + static_cast<float>(i) * angle_increment;
            geometry_msgs::msg::Point32 pt;
            pt.x = r * std::cos(angle);
            pt.y = r * std::sin(angle);
            pt.z = 0.0f;

            indices.push_back(i);
            points.push_back(pt);
        }

        if (indices.empty()) {
            // 无障碍物，发布空点云
            auto empty_cloud = std::make_unique<sensor_msgs::msg::PointCloud>();
            empty_cloud->header = scan_msg->header;
            publisher_->publish(std::move(empty_cloud));
            return;
        }

        // Step 2: 基于距离进行聚类（按原始扫描顺序遍历）
        std::vector<std::vector<size_t>> clusters;  // 存储每个聚类的索引
        std::vector<geometry_msgs::msg::Point32> cluster_points; // 辅助存储当前聚类的点（可选，但可直接用索引）

        // 第一个点开始新聚类
        clusters.push_back({indices[0]});
        std::vector<geometry_msgs::msg::Point32> current_cluster_points = {points[0]};

        for (size_t i = 1; i < indices.size(); ++i) {
            // 计算当前点与上一个点（属于当前聚类）的距离
            const auto& prev_pt = current_cluster_points.back();
            const auto& curr_pt = points[i];
            float dx = curr_pt.x - prev_pt.x;
            float dy = curr_pt.y - prev_pt.y;
            // 优化：使用平方距离比较，避免 std::sqrt 开销
            float dist = dx*dx + dy*dy;

            if (dist < cluster_distance_threshold_) {
                // 属于同一聚类
                clusters.back().push_back(indices[i]);
                current_cluster_points.push_back(curr_pt);
            } else {
                // 开始新聚类
                clusters.push_back({indices[i]});
                current_cluster_points = {curr_pt};
            }
        }

        // Step 3: 处理环形首尾合并（如果第一个点和最后一个点距离很近）
        if (clusters.size() >= 2) {
            const auto& first_pt = points.front();       // 对应 clusters[0][0] 的坐标
            const auto& last_pt = points.back();         // 对应 clusters.back().back() 的坐标

            float dx = first_pt.x - last_pt.x;
            float dy = first_pt.y - last_pt.y;
            
            // 优化：使用平方距离比较，避免 std::sqrt 开销
            float dist_sq = dx*dx + dy*dy;
            float threshold_sq = cluster_distance_threshold_;

            if (dist_sq < threshold_sq) {
                clusters[0].insert(clusters[0].end(), clusters.back().begin(), clusters.back().end());
                clusters.pop_back();
            }
        }

        // Step 4: 提取每个聚类的中心点并发布
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud>();
        cloud_msg->header = scan_msg->header;

        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            // 取聚类中间索引的点作为中心
            size_t mid_index = cluster[cluster.size() / 2];
            float r = ranges[mid_index];
            if (!std::isfinite(r)) continue;  // 理论上不应发生，因为之前筛选过

            float angle = angle_min + static_cast<float>(mid_index) * angle_increment;
            geometry_msgs::msg::Point32 pt;
            pt.x = r * std::cos(angle);
            pt.y = r * std::sin(angle);
            pt.z = 0.0f;
            cloud_msg->points.push_back(pt);
        }

        publisher_->publish(std::move(cloud_msg));
    }

    std::string robot_namespace_;
    double obstacle_threshold_;
    double cluster_distance_threshold_;  // 聚类距离阈值

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