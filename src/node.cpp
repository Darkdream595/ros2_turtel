#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;

const double pi = M_PI;

class Control : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_move;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn_start;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_;

    int count;
    double x, y, theta;

public:
    void publish_move_callback();
    void publish_turnshun_callback();
    void publish_turnni_callback();
    void run();
    Control(std::string name) : Node(name), tf_broadcaster_(this), x(0.0), y(0.0), theta(0.0)
    {
        count = 0;
        publisher_turn_start = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        publisher_move = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // 定时器，用于更新tf
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 每100毫秒更新一次
            std::bind(&Control::broadcast_transform, this));

        run();
    }

    void broadcast_transform()
    {
        // 创建并设置变换
        geometry_msgs::msg::TransformStamped transform_;
        transform_.header.stamp = this->now();
        transform_.header.frame_id = "map";   // 参考框架
        transform_.child_frame_id = "turtle"; // 子框架
        transform_.transform.translation.x = x;
        transform_.transform.translation.y = y;
        transform_.transform.translation.z = 0.0;

        // 计算四元数并手动设置
        double sin_half_theta = sin(theta / 2);
        double cos_half_theta = cos(theta / 2);

        transform_.transform.rotation.x = 0;
        transform_.transform.rotation.y = 0;
        transform_.transform.rotation.z = sin_half_theta;
        transform_.transform.rotation.w = cos_half_theta;

        // 发布变换
        tf_broadcaster_.sendTransform(transform_);
    }
};

void Control::publish_move_callback()
{
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 2.0;
    msg->angular.z = 0.0;
    publisher_move->publish(*msg);
    x += 2.0 * cos(theta); // 更新x坐标
    y += 2.0 * sin(theta); // 更新y坐标
    RCLCPP_INFO(this->get_logger(), "move.");
}

void Control::publish_turnshun_callback()
{
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->angular.z = -2.0 * pi / 3;
    publisher_turn_start->publish(*msg);
    theta += -2.0 * pi / 3; // 更新theta
    RCLCPP_INFO(this->get_logger(), "turn.");
}

void Control::publish_turnni_callback()
{
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->angular.z = pi / 3;
    publisher_turn_start->publish(*msg);
    theta += pi / 3; // 更新theta
    RCLCPP_INFO(this->get_logger(), "turn.");
}

void Control::run()
{
    std::this_thread::sleep_for(std::chrono::seconds(3));
    publish_turnshun_callback();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    publish_move_callback();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    count++;
    while (count <= 11)
    {
        broadcast_transform(); // 每次循环都发布变换
        if (count % 2 != 0)
        {
            publish_turnshun_callback();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            publish_move_callback();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            count++;
        }
        else
        {
            publish_turnni_callback();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            publish_move_callback();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            count++;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Control>("test");
    rclcpp::spin(node); // 修改为 spin，而不是 shutdown
    rclcpp::shutdown();
    return 0;
}
