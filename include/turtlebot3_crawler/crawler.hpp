#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

class Crawler:public rclcpp::Node{
public:
    Crawler();
    ~Crawler();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;
    int isNearObstacle(const std::vector<float> &) const;
    void publishStop() const;
    void publishGoStraight() const;
    void publishTurn() const;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};