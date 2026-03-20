#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <memory>

class LocalGoalToCmdVel : public rclcpp::Node
{
public:
    LocalGoalToCmdVel() : Node("local_goal_to_cmd_vel")
    {
        // 声明并获取参数
        this->declare_parameter<double>("v_max", 0.5);           // 最大线速度 (m/s)
        this->declare_parameter<double>("omega_max", 1.0);       // 最大角速度 (rad/s)
        this->declare_parameter<double>("angle_threshold", 0.1); // 允许直行的角度误差阈值 (rad)
        this->declare_parameter<double>("k_angular", 2.0);       // 角速度比例增益
        this->declare_parameter<bool>("stop", false);            // 停车开关，动态可改

        v_max_ = this->get_parameter("v_max").as_double();
        omega_max_ = this->get_parameter("omega_max").as_double();
        angle_threshold_ = this->get_parameter("angle_threshold").as_double();
        k_angular_ = this->get_parameter("k_angular").as_double();
        // stop 参数不存为成员变量，通过回调或每次读取获取最新值

        // 订阅局部目标点话题 (通常为 "/local_goal")
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("local_goal", 10,
            std::bind(&LocalGoalToCmdVel::goal_callback, this, std::placeholders::_1));

        // 发布小车速度指令话题 (通常为 "/cmd_vel")
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("car_2_cmd_vel", 10);

        // 设置参数回调，监听 stop 参数的变化
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        stop_cb_handle_ = param_subscriber_->add_parameter_callback(
            "stop", [this](const rclcpp::Parameter & p) {
                bool stop = p.as_bool();
                if (stop) {
                    // 立即发布停车指令，确保即使没有新目标点也能停车
                    auto twist = geometry_msgs::msg::Twist();
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    publisher_->publish(twist);
                    RCLCPP_INFO(this->get_logger(), "Stop parameter set to true, robot stopped.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Stop parameter set to false, resuming normal operation.");
                }
            });

        RCLCPP_INFO(this->get_logger(), "LocalGoalToCmdVel node initialized.");
    }

private:
    void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // 每次收到目标点时检查 stop 参数
        bool stop = false;
        this->get_parameter("stop", stop);
        if (stop) {
            // 如果 stop 为 true，则直接停车并忽略目标点
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
            return;
        }

        double x = msg->point.x;
        double y = msg->point.y;

        // 处理零向量情况：停车（但此时 stop 为 false，故此处仅处理目标点本身为零的情况）
        if (x == 0.0 && y == 0.0) {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
            return;
        }

        // 计算期望方向相对于车头（x轴）的角度
        double angle = std::atan2(y, x);  // 范围 [-π, π]

        // 角速度比例控制，并限幅
        double omega = k_angular_ * angle;
        if (omega > omega_max_) omega = omega_max_;
        else if (omega < -omega_max_) omega = -omega_max_;

        // 线速度控制：角度误差小时全速前进，误差大时减速
        double v = 0.0;
        double abs_angle = std::abs(angle);
        if (abs_angle < angle_threshold_) {
            v = v_max_;
        } else {
            // 平滑减速，最大减速至0（当角度接近π时）
            v = v_max_ * (1.0 - std::min(abs_angle / 3.1415, 1.0));
        }

        // 发布速度指令
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = v;
        twist.angular.z = omega;
        publisher_->publish(twist);
    }

    // 参数
    double v_max_;
    double omega_max_;
    double angle_threshold_;
    double k_angular_;

    // ROS2 通信成员
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // 参数事件处理
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> stop_cb_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalGoalToCmdVel>());
    rclcpp::shutdown();
    return 0;
}