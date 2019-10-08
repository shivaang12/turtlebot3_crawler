#include "turtlebot3_crawler/crawler.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

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

    publishStop();
}

Crawler::~Crawler(){

    publishStop();
}

void Crawler::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
const {
    RCLCPP_INFO(this->get_logger(), "I AM BEING CALLED");

    if(isNearObstacle(msg->ranges)){
        publishTurn();
    } else {
        publishGoStraight();
    }
}

int Crawler::isNearObstacle(const std::vector<float> &ranges) const {
    for(int i=315; i<360; i++){
        if(ranges[i] < 0.45){
            return 1;
        }
    }
    for(int i=0; i<45; i++){
        if(ranges[i] < 0.45){
            return 1;
        }
    }
    return 0;
}

void Crawler::publishStop() const{
    auto twist_trans = geometry_msgs::msg::Twist();
    twist_trans.linear.x = 0;
    twist_trans.linear.y = 0;
    twist_trans.linear.z = 0;
    twist_trans.angular.x = 0;
    twist_trans.angular.y = 0;
    twist_trans.angular.z = 0;

    pub_->publish(twist_trans);
}

void Crawler::publishGoStraight() const{
    auto twist_trans = geometry_msgs::msg::Twist();
    twist_trans.linear.x = 0.2;
    twist_trans.linear.y = 0;
    twist_trans.linear.z = 0;
    twist_trans.angular.x = 0;
    twist_trans.angular.y = 0;
    twist_trans.angular.z = 0;

    pub_->publish(twist_trans);
}

void Crawler::publishTurn() const{
    auto twist_trans = geometry_msgs::msg::Twist();
    twist_trans.linear.x = 0;
    twist_trans.linear.y = 0;
    twist_trans.linear.z = 0;
    twist_trans.angular.x = 0;
    twist_trans.angular.y = 0;
    twist_trans.angular.z = 0.1;

    pub_->publish(twist_trans);
}