#include "turtlebot3_crawler/crawler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Crawler>());
    rclcpp::shutdown();
    return 0;
}