#include "turtlebot3_crawler/crawler.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

Crawler::Crawler():Node("Crawler"){
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10)
    );

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        rclcpp::SensorDataQoS(),
        std::bind(&Crawler::laserCallback, this, std::placeholders::_1)
    );
}

void Crawler::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I AM BEING CALLED");
}