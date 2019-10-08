/**
 * @file    crawler.cpp
 * @author  Shivang Patel
 * @copyright   MIT License (c) 2019 Shivang Patel
 * 
 * @brief DESCRIPTION
 * This file contains class implementation of turtlebot3 crawler.
 * 
 * Copyright 2019 Shivang Patel
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

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

    // Check for obstacles
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

    // No obtacles founds
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