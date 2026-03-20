#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <mutex>

// 参数
const double l1 = 1.0;    // f1 的衰减速率
const double l2 = 1.0;    // f2 的增长速率
const double r  = 0.8;    // 反应向量场常数,反应边界在车距离障碍物为 r 的距离
const double c_p  = 0.3;
const double c  = -r + c_p;   // 排斥向量场常数,排斥边界在车距离障碍物为 c_p 的距离
const double eps = l2 * c / (l1 + l2);  // 扰动向量场参数

// 零出函数
double f1(double phi) {
    phi = phi - r; // 转换为反应边界
    if (phi > c) {
        return std::exp(l1 / (c - phi));
    }
    return 0.0;
}

// 零入函数
double f2(double phi) {
    phi = phi - r;
    if (phi < 0.0) {
        return std::exp(l2 / phi);
    }
    return 0.0;
}

// 凸包函数 UQ(phi) = f1 / (f1 + f2)
double UQ(double phi) {
    double val_f1 = f1(phi);
    double val_f2 = f2(phi);
    double denom = val_f1 + val_f2;
    return val_f1 / denom;
}

// 凸包函数 UR(phi) = f2 / (f1 + f2)
double UR(double phi) {
    double val_f1 = f1(phi);
    double val_f2 = f2(phi);
    double denom = val_f1 + val_f2;
    return val_f2 / denom;
}

class GoalPubNode : public rclcpp::Node
{
public:
    GoalPubNode(): Node("goal_pub")
    {
        // 声明参数
        this->declare_parameter<std::string>("namespace", "");
        std::string robot_namespace = this->get_parameter("namespace").as_string();
        this->declare_parameter<double>("goal_x", 0.0);
        this->declare_parameter<double>("goal_y", 0.0);
        goal_x_global_ = this->get_parameter("goal_x").as_double();
        goal_y_global_ = this->get_parameter("goal_y").as_double();


        // 构建话题名称
        std::string obstacle_topic = "/obstacle_centers";
        std::string odom_topic = "/odom";
        std::string local_goal_topic = "/local_goal";

        // 创建发布器
        local_target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(local_goal_topic, rclcpp::QoS(10));

        // 创建订阅器
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(obstacle_topic, rclcpp::QoS(10),
            std::bind(&GoalPubNode::obstacleCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10),
            std::bind(&GoalPubNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "GoalPubNode initialized with goal (%.3f, %.3f)", goal_x_global_, goal_y_global_);
    }

private:
    // 成员变量
    bool initialized_ = false;
    bool reached_ = false;
    double init_x_ = 0.0, init_y_ = 0.0, init_yaw_ = 0.0;
    double goal_x_global_, goal_y_global_;
    sensor_msgs::msg::PointCloud::ConstSharedPtr latest_obstacles_ = nullptr;
    std::mutex obstacles_mutex_;  // 保护 latest_obstacles_
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // 障碍物回调
    void obstacleCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        latest_obstacles_ = msg;
    }

    // 里程计回调（核心算法）
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 实时位置和朝向
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = tf2::getYaw(msg->pose.pose.orientation);

        // 第一次回调：记录初始位姿
        if (!initialized_) {
            init_x_ = x;
            init_y_ = y;
            init_yaw_ = yaw;
            RCLCPP_INFO(this->get_logger(), "Initial pose recorded: (%.3f, %.3f), yaw=%.3f rad", init_x_, init_y_, init_yaw_);
            RCLCPP_INFO(this->get_logger(), "Global goal: (%.3f, %.3f)", goal_x_global_, goal_y_global_);
            initialized_ = true;
        }

        // 如果已经到达，不再处理
        if (reached_) {
            return;
        }

        // 计算小车到全局目标点的向量
        double dx_global = goal_x_global_ - x;
        double dy_global = goal_y_global_ - y;
        double distance_to_goal = std::sqrt(dx_global * dx_global + dy_global * dy_global);

        // 判断是否接近目标（阈值 0.1 米）
        if (distance_to_goal < 0.1) {
            RCLCPP_INFO(this->get_logger(), "Reached the goal!");
            reached_ = true;
            return;
        }

        // --------------------------------------------------路径向量--------------------------------------------------
        const double r0 = 1.0;  // 路径跟随收敛参数

        // 1、计算路径参数
        double path_dx = goal_x_global_ - init_x_;
        double path_dy = goal_y_global_ - init_y_;
        double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);

        // 2、计算小车当前位置到初始-目标连线的垂直距离
        double distance_to_path;
        if (path_length < 1e-6) {
            // 初始点和目标点重合：距离就是到该点的距离
            distance_to_path = std::sqrt((x - init_x_) * (x - init_x_) + (y - init_y_) * (y - init_y_));
        } else {
            // 点到直线距离公式
            double cross = path_dx * (y - init_y_) - path_dy * (x - init_x_);
            distance_to_path = cross / path_length;   // 距离大于0表示点在线的左侧
        }

        // 3、计算phi_path向量场
        double unit_path_x = path_dx / path_length;   // 单位路径向量
        double unit_path_y = path_dy / path_length;

        double unit_perp_x =  unit_path_y;            // 单位垂直向量（顺时针旋转90度）
        double unit_perp_y = -unit_path_x;

        double phi_path_x_global = unit_path_x + r0 * distance_to_path * unit_perp_x;
        double phi_path_y_global = unit_path_y + r0 * distance_to_path * unit_perp_y;

        // 4、转化到小车坐标系下
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        double phi_path_x = phi_path_x_global * cos_yaw + phi_path_y_global * sin_yaw;
        double phi_path_y = -phi_path_x_global * sin_yaw + phi_path_y_global * cos_yaw;

        // 5、归一化
        double path_norm = std::sqrt(phi_path_x * phi_path_x + phi_path_y * phi_path_y);
        double unit_phi_path_x = (path_norm > 1e-6) ? phi_path_x / path_norm : 0.0;
        double unit_phi_path_y = (path_norm > 1e-6) ? phi_path_y / path_norm : 0.0;

        // --------------------------------------------------反应向量场与复合向量场--------------------------------------------------
        const int kr1 = 1;                       // 排斥参数（正数）
        double phi_r_x = 0.0;                    // 反应向量场累加
        double phi_r_y = 0.0;
        double UQ_p = 1.0;                        // 路径零入函数的累乘

        // 使用互斥锁安全获取最新障碍物
        sensor_msgs::msg::PointCloud::ConstSharedPtr obstacles;
        {
            std::lock_guard<std::mutex> lock(obstacles_mutex_);
            obstacles = latest_obstacles_;
        }

        if (obstacles && !obstacles->points.empty()) {
            for (const auto& pt : obstacles->points) {
                double obs_x = pt.x;
                double obs_y = pt.y;
                double dist = std::sqrt(obs_x * obs_x + obs_y * obs_y);

                if (dist < 1e-6) continue;

                // 小车指向障碍物的单位向量
                double unit_obs_x = obs_x / dist;
                double unit_obs_y = obs_y / dist;

                // 根据障碍物 y 坐标选择垂直方向（原 ROS1 逻辑）
                double unit_perp_obs_x = unit_obs_y;    // 顺时针旋转90度
                double unit_perp_obs_y = -unit_obs_x;
                if (obs_y <= 0) {
                    // 逆时针旋转90度
                    unit_perp_obs_x = -unit_obs_y;
                    unit_perp_obs_y =  unit_obs_x;
                }

                // 单个原始反应向量场
                double phi_r_x_i = unit_perp_obs_x + kr1 * (dist-r) * unit_obs_x;
                double phi_r_y_i = unit_perp_obs_y + kr1 * (dist-r) * unit_obs_y;

                // 归一化
                double i_norm = std::sqrt(phi_r_x_i * phi_r_x_i + phi_r_y_i * phi_r_y_i);
                double unit_phi_r_x = (i_norm > 1e-6) ? phi_r_x_i / i_norm : 0.0;
                double unit_phi_r_y = (i_norm > 1e-6) ? phi_r_y_i / i_norm : 0.0;

                // 累计反应向量场（乘以 UR）
                phi_r_x += UR(dist) * unit_phi_r_x;
                phi_r_y += UR(dist) * unit_phi_r_y;

                // 累乘路径零入函数
                UQ_p *= UQ(dist);
            }
        }

        // 复合向量场
        double phi_c_x = 0.0, phi_c_y = 0.0;
        if (obstacles && !obstacles->points.empty()) {
            phi_c_x = UQ_p * unit_phi_path_x + phi_r_x;
            phi_c_y = UQ_p * unit_phi_path_y + phi_r_y;
        } else {
            // 没有障碍物时直接使用路径向量
            phi_c_x = unit_phi_path_x;
            phi_c_y = unit_phi_path_y;
        }

        double magnitude = std::sqrt(phi_c_x * phi_c_x + phi_c_y * phi_c_y);
        double unit_phi_c_x = (magnitude > 1e-6) ? phi_c_x / magnitude : 0.0;
        double unit_phi_c_y = (magnitude > 1e-6) ? phi_c_y / magnitude : 0.0;

        // 构造并发布局部目标点（小车坐标系下的单位方向向量）
        geometry_msgs::msg::PointStamped local_target;
        local_target.header.stamp = this->get_clock()->now();
        local_target.header.frame_id = "base_footprint";   // 小车坐标系
        local_target.point.x = unit_phi_c_x;
        local_target.point.y = unit_phi_c_y;
        local_target.point.z = 0.0;

        local_target_pub_->publish(local_target);
    }
};

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<GoalPubNode>();

    // 运行
    rclcpp::spin(node);

    // 关闭
    rclcpp::shutdown();
    return 0;
}