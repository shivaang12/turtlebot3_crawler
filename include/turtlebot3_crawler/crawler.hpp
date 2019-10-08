/**
 * @file    crawler.hpp
 * @author  Shivang Patel
 * @copyright   MIT License (c) 2019 Shivang Patel
 * 
 * @brief DESCRIPTION
 * This header file contains class declaration of turtlebot3 crawler.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>


/**
 * @brief   Class which implements Turtlebot3 Crawler.
 */
class Crawler:public rclcpp::Node{
public:
    /**
     * @brief   This is a default constructor of this class.
     * 
     * @params[in]  None
     * 
     * @return  None
     */
    Crawler();

    /**
     * @brief   This is a destructor of this class.
     * 
     * @params[in]  None
     * 
     * @return  None
     */
    ~Crawler();

private:

    /**
     * @brief   A Callback function for subscriber which has subscribed '/scan'
     *  topic.
     * 
     * @params[in]  const LaserScan::SharedPtr  Data from the TB3 scan.
     * 
     * @return  void
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    /**
     * @brief   This method checks the obstacle in front of the TB3.
     * 
     * @params[in]  const std::vector<float>&   Data from the TB3 scan.
     * 
     * @return  int 1 if obstacle, or 0 if no obstacle
     */
    int isNearObstacle(const std::vector<float> &) const;

    /**
     * @brief   This method publishes the Twist message which will stop the
     * turtlebot3.
     * 
     * @params[in]  void.
     * 
     * @return  void
     */
    void publishStop() const;

    /**
     * @brief   This method publishes the Twist message which allow the
     * turtlebot3 to move forward.
     * 
     * @params[in]  void.
     * 
     * @return  void
     */
    void publishGoStraight() const;

    /**
     * @brief   This method publishes the Twist message which will make the
     * turtlebot3 to turn at its own place.
     * 
     * @params[in]  void.
     * 
     * @return  void
     */
    void publishTurn() const;

    //> Publisher which publishes the velocities for TB3
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    //> Subscriber which subscribes to the laser scan data of TB3
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};