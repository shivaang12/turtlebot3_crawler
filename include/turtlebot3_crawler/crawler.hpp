#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Crawler:public rclcpp::Node{
public:
    Crawler();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};