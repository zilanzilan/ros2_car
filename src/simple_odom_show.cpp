#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iomanip>

class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber(): Node("odom_subscriber_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,
            std::bind(&OdomSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 提取位置信息
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        // 打印实时信息
        RCLCPP_INFO(this->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    // 运行节点
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}