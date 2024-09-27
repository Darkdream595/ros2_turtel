#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
using namespace std;

const double pi = M_PI;

class Control : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_move;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn_start;
    rclcpp::TimerBase::SharedPtr timer_;

    int count;

public:
    void publish_move_callback();
    void publish_turnshun_callback();
    void publish_turnni_callback();
    void run();
    Control(std::string name) : Node(name)
    {
        count = 0;
        publisher_turn_start = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        publisher_move = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        run();
        // timer_ = this->create_wall_timer(
        //     500ms, // 定时器周期
        //     std::bind(&Control::run, this));
    }
};

void Control::publish_move_callback()
{

    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 2.0;
    msg->angular.z = 0.0;
    publisher_move->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "move.");
}

void Control::publish_turnshun_callback()
{

    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->angular.z = -2.0 * pi / 3;
    publisher_turn_start->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "turn.");
}

void Control::publish_turnni_callback()
{
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->angular.z = pi / 3;
    publisher_turn_start->publish(*msg);
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
    while (count <= 12)
    {
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
    rclcpp::shutdown();
    return 0;
}
