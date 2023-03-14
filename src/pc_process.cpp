#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/point_cloud.hpp>
// #include <sensor_msgs/point_cloud_conversion.hpp>
// #include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include "cuda_pointpillars_ros/cuda_pp_ros.hpp"

using sensor_msgs::msg::PointCloud2;

typedef sensor_msgs::msg::PointCloud2 PointCloud2;

class MsgExchangeClass : public rclcpp::Node
{
private:
  /* data */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  // rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_;

  void callback(const PointCloud2::SharedPtr msg){
    infer(msg);
  }
public:
  MsgExchangeClass():Node("pc_process")
  {
    sub_ = this->create_subscription<PointCloud2>("rslidar_points", 10, std::bind(&MsgExchangeClass::callback, this, std::placeholders::_1));
    // pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("objects", 10);
  }
};


int main(int argc, char ** argv)
{
  Getinfo();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgExchangeClass>());
  rclcpp::shutdown();
  return 0;
}
